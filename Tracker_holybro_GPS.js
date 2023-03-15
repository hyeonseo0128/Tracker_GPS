const { SerialPort } = require('serialport');
const mqtt = require('mqtt');
const { nanoid } = require("nanoid");
const moment = require('moment');

const mavlink = require('./mavlink.js');

const gpi_frequency = 2;

let boot_time = null;
let boot_start_time = moment().valueOf();

// let uart_port_num = '/dev/ttyUSB1';
// let pcPortNum = 'COM10';
// let pcBaudrate = '115200';
// let pcPort = null;

let gpsPortNum = 'COM3';
let gpsBaudrate = '115200';
let gpsPort = null;

let _msg = '';
const my_system_id = 254;

let NAV_PVT = {};
NAV_PVT.header = null;
NAV_PVT.class = null;
NAV_PVT.ID = null;
NAV_PVT.length = null;
NAV_PVT.fixType = null;
NAV_PVT.flags = null;
NAV_PVT.lon = null;
NAV_PVT.lat = null;
NAV_PVT.height = null;
NAV_PVT.hMSL = null;
NAV_PVT.hAcc = null;
NAV_PVT.gSpeed = null;
NAV_PVT.headMot = null;

let mavData = {};
mavData.lat = 0;
mavData.lon = 0;
mavData.alt = 0;
mavData.relative_alt = 0;
mavData.vx = 0;
mavData.hdg = 0;

let local_mqtt_client = null;
let pub_tracker_gps_location_topic = '/GPS/location';
let pub_tracker_gps_state_topic = '/GPS/state'

let t_count = 0;
let gps_state = null;

local_mqtt_connect('127.0.0.1');

// pcPortOpening();
gpsPortOpening();

//------------- gps uart communication -------------
function gpsPortOpening() {
    if (gpsPort === null) {
        gpsPort = new SerialPort({
            path: gpsPortNum,
            baudRate: parseInt(gpsBaudrate, 10),
        });
        gpsPort.on('open', gpsPortOpen);
        gpsPort.on('close', gpsPortClose);
        gpsPort.on('error', gpsPortError);
        gpsPort.on('data', gpsPortData);
    } else {
        if (gpsPort.isOpen) {
            gpsPort.close();
            gpsPort = null;
            setTimeout(gpsPortOpening, 2000);
        } else {
            gpsPort.open();
        }
    }
}

function gpsPortOpen() {
    console.log('gpsPort(' + gpsPort.path + '), gpsPort rate: ' + gpsPort.baudRate + ' open.');
}

function gpsPortClose() {
    console.log('gpsPort closed.');

    setTimeout(gpsPortOpening, 2000);
}

function gpsPortError(error) {
    console.log('[gpsPort error]: ' + error.message);

    setTimeout(gpsPortOpening, 2000);
}

function gpsPortData(data) {
    // console.log('from gps:', data.toString('hex'));
    t_count = 0;
    _msg += data.toString('hex');

    try {
        NAV_PVT.header = _msg.substring(0, 4);
        NAV_PVT.class = _msg.substring(4, 6);
        NAV_PVT.ID = _msg.substring(6, 8);
        NAV_PVT.length = parseInt(_msg.substring(8, 10), 16);

        if (NAV_PVT.header === 'b562' && NAV_PVT.class === '01' && NAV_PVT.ID === '07') {
            NAV_PVT.payload = NAV_PVT.payload = _msg.substring(12, 10 + (NAV_PVT.length * 2));
            console.log('NAV_PVT.payload: ' + NAV_PVT.payload);

            NAV_PVT.fixType = hexParser(NAV_PVT.payload.substring(40, 42), 'uint8');
            NAV_PVT.flags = hexParser(NAV_PVT.payload.substring(42, 44), 'binary');
            NAV_PVT.lon = hexParser(NAV_PVT.payload.substring(48, 56), 'int32le') / 10000000;
            NAV_PVT.lat = hexParser(NAV_PVT.payload.substring(56, 64), 'int32le') / 10000000;
            NAV_PVT.height = hexParser(NAV_PVT.payload.substring(64, 72), 'int32le') / 1000;
            NAV_PVT.hMSL = hexParser(NAV_PVT.payload.substring(72, 80), 'int32le') / 1000;
            NAV_PVT.hAcc = hexParser(NAV_PVT.payload.substring(80, 88), 'uint32le') / 1000;
            NAV_PVT.gSpeed = hexParser(NAV_PVT.payload.substring(120, 128), 'int32le') / 1000;
            NAV_PVT.headMot = hexParser(NAV_PVT.payload.substring(128, 136), 'int32le') / 100000;

            console.log('NAV_PVT.fixType: ' + NAV_PVT.fixType);
            console.log('NAV_PVT.flags: ' + NAV_PVT.flags);
            console.log('NAV_PVT.lon: ' + NAV_PVT.lon);
            console.log('NAV_PVT.lat: ' + NAV_PVT.lat);
            console.log('NAV_PVT.height: ' + NAV_PVT.height);
            console.log('NAV_PVT.hMSL: ' + NAV_PVT.hMSL);
            console.log('NAV_PVT.headMot: ' + NAV_PVT.headMot);

            if (NAV_PVT.fixType === 3 && NAV_PVT.flags === '00000001') {
                gps_state = 'done'
                mavData.lat = NAV_PVT.lat;
                mavData.lon = NAV_PVT.lon;
                mavData.alt = NAV_PVT.height;
                mavData.relative_alt = NAV_PVT.hMSL;
                mavData.vx = NAV_PVT.gSpeed;
                mavData.hdg = NAV_PVT.headMot;
            } else {
                gps_state = 'no fix'
                mavData.lat = 0;
                mavData.lon = 0;
                mavData.alt = 0;
                mavData.relative_alt = 0;
                mavData.vx = 0;
                mavData.hdg = 0;
            }

            _msg = _msg.substring(200, _msg.length);
        } else {
            _msg = _msg.substring(2, _msg.length);
        }
    } catch (err) {
        console.log('GPS message parse error -> ' + err);
    }



    if (pcPort !== null) { // send gimbal data to pc(U-center application)
        pcPort.write(data);
    }
}
//---------------------------------------------------





//------------- uart communication -------------
function pcPortOpening() {
    if (pcPort === null) {
        pcPort = new SerialPort({
            path: pcPortNum,
            baudRate: parseInt(pcBaudrate, 10),
        });
        pcPort.on('open', pcPortOpen);
        pcPort.on('close', pcPortClose);
        pcPort.on('error', pcPortError);
        pcPort.on('data', pcPortData);
    } else {
        if (pcPort.isOpen) {
            pcPort.close();
            pcPort = null;
            setTimeout(pcPortOpening, 2000);
        } else {
            pcPort.open();
        }
    }
}

function pcPortOpen() {
    console.log('pcPort(' + pcPort.path + '), pcPort rate: ' + pcPort.baudRate + ' open.');
}

function pcPortClose() {
    console.log('pcPort closed.');

    setTimeout(pcPortOpening, 2000);
}

function pcPortError(error) {
    console.log('[pcPort error]: ' + error.message);

    setTimeout(pcPortOpening, 2000);
}

function pcPortData(data) {
    console.log('from PC:', data.toString('hex'));
    if (gpsPort !== null) {
        gpsPort.write(data);
    }
}
//---------------------------------------------------

function hexParser(data, type) {
    switch (type.toLowerCase()) {
        case 'int8':
            return Buffer.from(data, 'hex').readInt8(0);
        case 'int16le':
            return Buffer.from(data, 'hex').readInt16LE(0);
        case 'int16be':
            return Buffer.from(data, 'hex').readInt16BE(0);
        case 'int32le':
            return Buffer.from(data, 'hex').readInt32LE(0);
        case 'int32be':
            return Buffer.from(data, 'hex').readInt32BE(0);
        case 'uint8':
            return Buffer.from(data, 'hex').readUInt8(0);
        case 'uint16le':
            return Buffer.from(data, 'hex').readUInt16LE(0);
        case 'uint16be':
            return Buffer.from(data, 'hex').readUInt16BE(0);
        case 'uint32le':
            return Buffer.from(data, 'hex').readUInt32LE(0);
        case 'uint32be':
            return Buffer.from(data, 'hex').readUInt32BE(0);
        case 'binary':
            return parseInt(data, 16).toString(2).padStart(8, '0');

        default:
            console.log('Invalid data type --> ' + type);
    }
}


function local_mqtt_connect(broker_ip) {
    if (local_mqtt_client == null) {
        var connectOptions = {
            host: broker_ip,
            port: 1883,
            protocol: "mqtt",
            keepalive: 10,
            clientId: 'Tracker_GPS_' + nanoid(15),
            protocolId: "GPS_",
            protocolVersion: 4,
            clean: true,
            reconnectPeriod: 2000,
            connectTimeout: 2000,
            rejectUnauthorized: false
        };


        local_mqtt_client = mqtt.connect(connectOptions);

        local_mqtt_client.on('connect', function () {
            console.log('[local_mqtt] connected to ' + broker_ip);
        });

        local_mqtt_client.on('error', function (err) {
            console.log('[local_mqtt] error: ' + err.message);
            local_mqtt_client = null;
            local_mqtt_connect(broker_ip);
        });
    }
}

setInterval(() => {
    local_mqtt_client.publish(pub_tracker_gps_state_topic, gps_state);
    t_count++;
    if (t_count > (30 * gpi_frequency)) {
        console.log("Couldn't receive messages.")
    } else {
        setTimeout(createMAVLinkData, 1, my_system_id, boot_time, mavData);
    }
}, (1000 / gpi_frequency));

setInterval(function () {
    boot_time = moment().valueOf() - boot_start_time;
}, 1);

function createMAVLinkData(sys_id, boot_time, mavdata) {
    // #33, GLOBAL_POSITION_INT
    let params = {};
    params.target_system = sys_id;
    params.target_component = 1;
    params.time_boot_ms = boot_time;
    params.lat = parseFloat(mavdata.lat) * 1E7;
    params.lon = parseFloat(mavdata.lon) * 1E7;
    params.alt = parseFloat(mavdata.alt) * 1000;
    params.relative_alt = 0;  // TODO: 추후 트래커 높이(고정값, 삼각대 높이)로 수정
    params.vx = mavdata.vx;
    params.vy = 0;
    params.vz = 0;
    params.hdg = mavdata.hdg;

    try {
        globalpositionint_msg = mavlinkGenerateMessage(params.target_system, params.target_component, mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, params);
        if (globalpositionint_msg === null) {
            console.log("mavlink message(MAVLINK_MSG_ID_GLOBAL_POSITION_INT) is null");
        } else {
            local_mqtt_client.publish(pub_tracker_gps_location_topic, Buffer.from(globalpositionint_msg, 'hex'));

        }
    } catch (ex) {
        console.log('[ERROR (GLOBAL_POSITION_INT)] ' + ex);
    }
}

function mavlinkGenerateMessage(src_sys_id, src_comp_id, type, params) {
    const mavlinkParser = new MAVLink(null/*logger*/, src_sys_id, src_comp_id);
    try {
        var mavMsg = null;
        var genMsg = null;

        switch (type) {
            case mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                mavMsg = new mavlink.messages.global_position_int(params.time_boot_ms,
                    params.lat,
                    params.lon,
                    params.alt,
                    params.relative_alt,
                    params.vx,
                    params.vy,
                    params.vz,
                    params.hdg
                );
                break;
        }
    } catch (e) {
        console.log('MAVLINK EX:' + e);
    }

    if (mavMsg) {
        genMsg = Buffer.from(mavMsg.pack(mavlinkParser));
    }

    return genMsg;
}