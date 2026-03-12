const exposes = require('zigbee-herdsman-converters/lib/exposes');
const e = exposes.presets;

let _learnedChunks = [];

const fzLocal = {
    ir_blaster_state: {
        cluster: '65280', // 0xFF00
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const payload = {};

            // Attr 0x0001 — send_ir_code state
            if (msg.data.hasOwnProperty(1) || msg.data.hasOwnProperty('1')) {
                let code = msg.data['1'] !== undefined ? msg.data['1'] : msg.data[1];
                if (Buffer.isBuffer(code)) code = code.toString('utf8');
                if (typeof code !== 'string') code = String(code);
                code = code.replace(/\0/g, '');

                // WORKAROUND: If the ESP32 reports an empty string, push a placeholder
                // to force the Z2M UI to overwrite the text box.
                payload.send_ir_code = code === '' ? '(ready)' : code;
            }

            // Attr 0x0002 — learn_mode (bool)
            if (msg.data.hasOwnProperty(2) || msg.data.hasOwnProperty('2')) {
                const val = msg.data['2'] !== undefined ? msg.data['2'] : msg.data[2];
                payload.learn_mode = val ? 'ON' : 'OFF';
            }

            // Attr 0x0003 — learned_code (chunked CSV: "N/T:data")
            if (msg.data.hasOwnProperty(3) || msg.data.hasOwnProperty('3')) {
                let code = msg.data['3'] !== undefined ? msg.data['3'] : msg.data[3];
                if (Buffer.isBuffer(code)) code = code.toString('utf8');
                if (typeof code !== 'string') code = String(code);
                code = code.replace(/\0/g, '');

                const chunkMatch = code.match(/^(\d+)\/(\d+):(.*)$/s);
                if (!chunkMatch) {
                    payload.learned_code = code;
                    return payload;
                }

                const n = parseInt(chunkMatch[1], 10);
                const t = parseInt(chunkMatch[2], 10);
                const data = chunkMatch[3];
                if (n === 0) _learnedChunks = [];
                _learnedChunks[n] = data;
                if (n + 1 === t) {
                    payload.learned_code = _learnedChunks.join(',');
                    _learnedChunks = [];
                }
                // intermediate chunks: emit no payload yet
            }

            return payload;
        },
    }
};

const CHUNK_SIZE = 60; // chars of CSV data per Zigbee write

const tzLocal = {
    ir_blaster_commands: {
        key: ['send_ir_code', 'learn_mode'],
        convertSet: async (entity, key, value, meta) => {
            if (key === 'send_ir_code') {
                const chunks = [];
                for (let i = 0; i < value.length; i += CHUNK_SIZE) {
                    chunks.push(value.slice(i, i + CHUNK_SIZE));
                }
                for (let i = 0; i < chunks.length; i++) {
                    const chunk = `${i}/${chunks.length}:${chunks[i]}`;
                    await entity.write(0xFF00, { 1: { value: chunk, type: 0x42 } });
                }
                return { state: { send_ir_code: '' } };
            }

            if (key === 'learn_mode') {
                const isLearning = typeof value === 'string'
                    ? (value.toUpperCase() === 'ON' || value.toUpperCase() === 'TRUE')
                    : (value === true || value === 1);
                await entity.write(0xFF00, { 2: { value: isLearning ? 1 : 0, type: 0x10 } });
                return { state: { learn_mode: isLearning ? 'ON' : 'OFF' } };
            }
        },
    }
};

const definition = {
    zigbeeModel: ['IR Blaster'],
    model: 'ESP32_IR_Blaster',
    vendor: 'ZigbeeHive',
    description: 'Custom ESP32 IR Blaster',
    fromZigbee: [fzLocal.ir_blaster_state],
    toZigbee: [tzLocal.ir_blaster_commands],
    configure: async (device, _coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);
        try {
            await endpoint.configureReporting('65280', [{
                attribute: { ID: 0x0003, type: 0x42 },
                minimumReportInterval: 0,
                maximumReportInterval: 65535,
                reportableChange: 0,
            }], { manufacturerCode: null });
        } catch (err) {
            console.warn(`[IR_BLASTER] Failed to configure reporting: ${err.message}`);
        }
    },
    exposes: [
        e.text('send_ir_code', exposes.access.SET)
            .withDescription('Send IR code. Paste the CSV string from learned_code here'),
        exposes.binary('learn_mode', exposes.access.STATE_SET, true, false)
            .withDescription('Toggle ON to learn 1 command'),
        e.text('learned_code', exposes.access.STATE)
            .withDescription('Last learned command as signed CSV (µs timings)'),
    ],
};

module.exports = definition;
