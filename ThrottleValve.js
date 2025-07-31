const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const readline = require('readline');


class Utils {
    static sleep(ms) {
        return new Promise((resolve) => {
            setTimeout(resolve, ms);
        });
    }
}
class ArduinoSender {
    constructor(uartPort) {
        this.uartPort = uartPort;
    }
    sendToArduino(command) {
        if (!this.uartPort.writable) {
            return console.error('port is not ready');
        }

        this.uartPort.write(command.trim() + '\n', (err) => {
            if (err) {
                return console.error('Error occurred while sending', err.message);
            }
            console.log('sent:', command);
        });
    }
}
class Valve {
    constructor(arduinoSender) {
        this.arduinoSender = arduinoSender;
    }
    maxPressure = 0.1; // бар
    maxAngle = 90; // градусов
    arduinoSender;
    min = -1;
    max = 1024;
    getMaxPressure() {
        return this.maxPressure;
    }
    getMaxAngle() {
        return this.maxAngle;
    }
    getMinBound() {
        return this.min;
    }
    getMaxBound() {
        return this.max;
    }
    setMinBound(min) {
        this.min = min;
    }
    setMaxBound(max) {
        this.max = max;
    }
    async init() {
        await Utils.sleep(3000);
        this.arduinoSender.sendToArduino("start");
        await Utils.sleep(3000);
        this.arduinoSender.sendToArduino("enable");
    }
}
class AnalogBoundaries {
    static maxAnalog = 1023;
    static minAnalog = 0;
}
class Main {
    constructor() {
        const uartPort = new SerialPort({
            path: 'COM3',
            baudRate: 9600,
            autoOpen: true, 
        });
        const parser = uartPort.pipe(new ReadlineParser({ delimiter: '\n' }));

        uartPort.on('open', () => {
            console.log('UART Port is opened');
        });

        uartPort.on('error', (err) => {
            console.error('Error occurred:', err.message);
        });
        var arduinoSender = new ArduinoSender(uartPort);
        var valve = new Valve(arduinoSender);
        valve.init();
        parser.on('data', (data) => {
            data = data.trim();
            console.log('Received:', data);

            var startPressureIndex = data.indexOf("(Bar):") + 7;
            var endPressureIndex = data.indexOf("KP") - 2;

            if (valve.getMinBound() < AnalogBoundaries.minAnalog) {
                var startMinIndex = data.indexOf("MIN_POSITION:");
                if (data.startsWith("MIN_POSITION")) {
                    var startMinIndex = startMinIndex + "MIN_POSITION".length + 2;
                    valve.setMinBound(parseInt(data.substring(startMinIndex, startMinIndex + 5)))
                }
            }

            if (valve.getMaxBound() > AnalogBoundaries.maxAnalog) {
                var startMaxIndex = data.indexOf("MAX_POSITION:");
                if (data.startsWith("MAX_POSITION")) {
                    var startMaxIndex = startMaxIndex + "MAX_POSITION".length + 2;
                    valve.setMaxBound(parseInt(data.substring(startMaxIndex, startMaxIndex + 5)));    
                }
            }
            
            if (startPressureIndex >= 0 && endPressureIndex >= 0 && endPressureIndex > startPressureIndex) {
                var bars = parseFloat(data.substring(startPressureIndex, endPressureIndex), 10);

                if (!isNaN(bars) && valve.getMaxBound() < AnalogBoundaries.maxAnalog + 1 && valve.getMinBound() > AnalogBoundaries.minAnalog - 1) {


                    const clampedPressure = Math.max(0, Math.min(bars, valve.getMaxPressure()));
                    const angle = (clampedPressure / valve.getMaxPressure()) * valve.getMaxAngle();

                    const analogValue = Math.round(valve.getMinBound() + (angle / valve.getMaxAngle()) * (valve.getMaxBound() - valve.getMinBound()));

                    console.log(`Pressure: ${clampedPressure.toFixed(2)} bar => Angle: ${angle.toFixed(2)}° => ${analogValue}`);
                    Utils.sleep(300);
                    
                    arduinoSender.sendToArduino(`set ${analogValue}`);
                    
                } else {
                    console.warn('Could not parse pressure from data:', data);
                }
            } else {
                console.warn('Pressure markers not found in data:', data);
            }
        });
        const rl = readline.createInterface({
            input: process.stdin,
            output: process.stdout
        });

        rl.on('line', (input) => {
            var arduinoSender = new ArduinoSender(uartPort);
            arduinoSender.sendToArduino(input);
        });
    }
}

new Main();