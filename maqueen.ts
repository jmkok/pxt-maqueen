/**
 * @file pxt-maqueen/maqueen.ts
 * @brief DFRobot's maqueen makecode library.
 * @n [Get the module here](https://www.dfrobot.com.cn/goods-1802.html)
 * @n This is a MakeCode graphical programming education robot.
 *
 * @copyright    [DFRobot](http://www.dfrobot.com), 2016
 * @copyright    MIT Lesser General Public License
 *
 * @author [email](jie.tang@dfrobot.com)
 * @version  V1.0.2
 * @date  2019-10-08
*/

let maqueencb: Action
let maqueenmycb: Action
let maqueene = "1"
let maqueenparam = 0
let alreadyInit = 0
let IrPressEvent = 0
const MOTER_ADDRESSS = 0x10

//% weight=10 color=#008B00 icon="\uf136" block="Maqueen"
namespace maqueen {

    export class Packeta {
        public mye: string;
        public myparam: number;
    }

	export enum Side {
		//% blockId="Left" block="left"
		Left = 1,
		//% blockId="Right" block="right"
		Right = 2,
		//% blockId="Both" block="both"
		Both = 3
	}

    export enum State {
        //% blockId="On" block="ON"
        On = 1,
        //% blockId="Off" block="OFF"
        Off = 0
    }

    export enum Servos {
        //% blockId="S1" block="S1"
        S1 = 0,
        //% blockId="S2" block="S2"
        S2 = 1
    }

	export enum PingUnit {
		//% block="cm"
		Centimeters,
		//% block="inch"
		Inches,
		//% block="μs"
		MicroSeconds
	}

    export enum Dir {
        //% blockId="CW" block="Forward"
        CW = 0x0,
        //% blockId="CCW" block="Backward"
        CCW = 0x1
    }

    //% advanced=true shim=maqueenIR::initIR
    function initIR(pin: Pins): void {
        return
    }

    //% advanced=true shim=maqueenIR::onPressEvent
    function onPressEvent(btn: RemoteButton, body: Action): void {
        return
    }

    //% advanced=true shim=maqueenIR::getParam
    function getParam(): number {
        return 0
    }

    function maqueenInit(): void {
        if (alreadyInit == 1) {
            return
        }
        initIR(Pins.P16)
        alreadyInit = 1
    }

    //% weight=100
    //% blockGap=50
    //% blockId=IR_callbackUser block="on IR received"
    export function IR_callbackUser(maqueencb: (message: number) => void) {
        maqueenInit();
        IR_callback(() => {
            const packet = new Packeta();
            packet.mye = maqueene;
            maqueenparam = getParam();
            packet.myparam = maqueenparam;
            maqueencb(packet.myparam);
        });
    }

    /**
     * Read IR sensor value.
     */

    //% weight=10
    //% blockId=IR_read block="read IR key"
    export function IR_read(): number {
        maqueenInit()
        return getParam()
    }

    /**
     * Read the version number.
     */

    //% weight=10
    //% blockId=IR_read_version block="get product information"
    export function IR_read_version(): string {
        maqueenInit()
        pins.i2cWriteNumber(0x10, 50, NumberFormat.UInt8BE);
        let dataLen = pins.i2cReadNumber(0x10, NumberFormat.UInt8BE);
        pins.i2cWriteNumber(0x10, 51, NumberFormat.UInt8BE);
        let buf = pins.i2cReadBuffer(0x10, dataLen, false);
        let version = "";
        for (let index = 0; index < dataLen; index++) {
            version += String.fromCharCode(buf[index])
        }
        return version
    }

    function IR_callback(a: Action): void {
        maqueencb = a
        IrPressEvent += 1
        onPressEvent(IrPressEvent, maqueencb)
    }

    /**
     * Read ultrasonic sensor.
     * see: https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
     * We could multiply us with 1.5 for more accurate values...
     */

    //% blockId=ultrasonic_sensor block="read ultrasonic sensor |%unit "
    //% weight=95
    export function Ultrasonic(unit: PingUnit, maxCmDistance = 500): number {
		let timeout = maxCmDistance * 58;
		pins.digitalWritePin(DigitalPin.P1, 1);
		basic.pause(1); /* todo: wait 10us */
		pins.digitalWritePin(DigitalPin.P1, 0);
		let us = pins.pulseIn(DigitalPin.P2, PulseValue.High, timeout);
        if (us <= 0 || us > timeout)
			us = timeout;
        switch (unit) {
            case PingUnit.Centimeters:
				return Math.round(us / 58);
            case PingUnit.Inches:
				return Math.round(us / 148);
            default:
				return us;
        }

    }

    /**
     * Set the direction and speed of Maqueen motor.
     */

    //% weight=90
    //% blockId=motor_MotorRun block="motor|%index|move|%Dir|at speed|%speed"
    //% speed.min=0 speed.max=255
    //% index.fieldEditor="gridpicker" index.fieldOptions.columns=2
    //% direction.fieldEditor="gridpicker" direction.fieldOptions.columns=2
    export function motorRun(side: Side, direction: Dir, speed: number): void {
        let buf = pins.createBuffer(3);
        if (side == Side.Left) {
            buf[0] = 0x00;
            buf[1] = direction;
            buf[2] = speed;
            pins.i2cWriteBuffer(0x10, buf);
        }
        else if (side == Side.Right) {
            buf[0] = 0x02;
            buf[1] = direction;
            buf[2] = speed;
            pins.i2cWriteBuffer(0x10, buf);
        }
        else if (side == Side.Both) {
            buf[0] = 0x00;
            buf[1] = direction;
            buf[2] = speed;
            pins.i2cWriteBuffer(0x10, buf);
            buf[0] = 0x02;
            pins.i2cWriteBuffer(0x10, buf);
        }
    }

    /**
     * Stop the Maqueen motor.
     */
    //% weight=20
    //% blockId=motor_motorStop block="motor |%motors stop"
    //% motors.fieldEditor="gridpicker" motors.fieldOptions.columns=2
    export function motorStop(side: Side): void {
        let buf = pins.createBuffer(3);
        if (side == Side.Left) {
            buf[0] = 0x00;
            buf[1] = 0;
            buf[2] = 0;
            pins.i2cWriteBuffer(0x10, buf);
        }
        else if (side == Side.Right) {
            buf[0] = 0x02;
            buf[1] = 0;
            buf[2] = 0;
            pins.i2cWriteBuffer(0x10, buf);
        }

        else if (side == Side.Both) {
            buf[0] = 0x00;
            buf[1] = 0;
            buf[2] = 0;
            pins.i2cWriteBuffer(0x10, buf);
            buf[0] = 0x02;
            pins.i2cWriteBuffer(0x10, buf);
        }

    }

    /**
     * Read line tracking sensor.
     */

    //% weight=20
    //% blockId=read_Patrol block="read |%patrol line tracking sensor"
    //% patrol.fieldEditor="gridpicker" patrol.fieldOptions.columns=2
    export function readPatrol(side: Side): number {
        if (side == Side.Left) {
            return pins.digitalReadPin(DigitalPin.P13)
        } else if (side == Side.Right) {
            return pins.digitalReadPin(DigitalPin.P14)
        } else {
            return -1
        }
    }

    /**
     * Turn on/off the LEDs.
     */

    //% weight=20
    //% blockId=writeLED block="LEDlight |%led turn |%ledswitch"
    //% led.fieldEditor="gridpicker" led.fieldOptions.columns=2
    //% ledswitch.fieldEditor="gridpicker" ledswitch.fieldOptions.columns=2
    export function writeLED(side: Side,state: State): void {
        if (side == Side.Left) {
            pins.digitalWritePin(DigitalPin.P8, state)
        } else if (side == Side.Right) {
            pins.digitalWritePin(DigitalPin.P12, state)
        } else if (side == Side.Both) {
            pins.digitalWritePin(DigitalPin.P8, state)
            pins.digitalWritePin(DigitalPin.P12, state)
        } else {
            return
        }
    }

    /**
     * Set the Maqueen servos.
     */

    //% weight=90
    //% blockId=servo_ServoRun block="servo|%index|angle|%angle"
    //% angle.min=0 angle.max=180
    //% index.fieldEditor="gridpicker" index.fieldOptions.columns=2
    export function servoRun(index: Servos, angle: number): void {
        let buf = pins.createBuffer(2);
        if (index == 0) {
            buf[0] = 0x14;
        }
        if (index == 1) {
            buf[0] = 0x15;
        }
        buf[1] = angle;
        pins.i2cWriteBuffer(0x10, buf);
    }

}
