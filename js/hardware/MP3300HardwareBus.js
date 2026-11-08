/**
 * MP3300HardwareBus.js
 * 
 * Hardware Emulation of Yaskawa Machine Controller MP3300 Memory & I/O Bus
 * Supports MPE720 Engineering Tool Register Specifications:
 * - MB (Bit/Byte), MW (16-bit Signed Word), ML (32-bit Long), MQ (64-bit Quad)
 * - MF (32-bit Float), MD (64-bit Double), MA (Memory Address Pointer)
 * - Local I/O Registers: IW00C80, IW00C81 (Local Inputs) and OW00C80, OW00C81 (Local Outputs)
 * - 32 Digital Inputs (DI_00..DI_31) and 32 Digital Outputs (DO_00..DO_31)
 * - Mapping to Robot Arm IN#1..8 and OT#1..8
 * - Built-in Default PLC Ladder Logic Engine
 */

export class MP3300HardwareBus {
  constructor(options = {}) {
    // 64 KB Main Data Memory (M-Registers)
    this.mBuffer = new ArrayBuffer(65536);
    this.mView = new DataView(this.mBuffer);

    // 256 Bytes I/O Buffer
    this.ioBuffer = new ArrayBuffer(256);
    this.ioView = new DataView(this.ioBuffer);

    // 1 KB System G-Registers (Flags for MSG-SNDE, etc.)
    this.gBuffer = new ArrayBuffer(1024);
    this.gView = new DataView(this.gBuffer);

    // Digital Inputs DI[0..31] and Outputs DO[0..31]
    this.DI = new Uint8Array(32);
    this.DO = new Uint8Array(32);

    // Robot Dedicated Signals IN#1..8 and OT#1..8
    this.robotIN = new Uint8Array(9); // 1-indexed (IN#1..IN#8)
    this.robotOT = new Uint8Array(9); // 1-indexed (OT#1..OT#8)

    // Robot Mode Key Switch: 0 = TEACH, 1 = PLAY, 2 = REMOTE
    this.robotMode = 2; // Default to REMOTE

    // Emergency Stop State (true = pressed/locked)
    this.emergencyStop = false;

    // Built-in Default PLC Ladder Logic State Machine
    this.builtinPLCOn = true;
    this.plcState = 'IDLE'; // IDLE, RUNNING_CONVEYOR, WAITING_ROBOT_PICK, ROBOT_PACKING, DISCHARGING_BOX
    this.plcBoxClusterCount = 0; // Clusters packed in current box (target = 2)

    // Listeners for UI state change notifications
    this.listeners = new Set();

    // Initialize default states
    this.initDefaultRegisters();
  }

  /**
   * Register change listener
   */
  subscribe(fn) {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  }

  notifyChange() {
    for (const fn of this.listeners) {
      fn(this);
    }
  }

  initDefaultRegisters() {
    // Initialize Local I/O Registers
    this.syncDIToIW();
    this.syncDOToOW();

    // Default MSG_SNDE parameter struct header at MA00400 (MW00400)
    // Structure: Channel, Connection Type (UDP), Remote IP, Remote Port
    this.writeMW(400, 1);       // Channel 1
    this.writeMW(401, 16);      // Dev-Typ: Ethernet (218IFD)
    this.writeMW(402, 10001);   // Local Port
    this.writeMW(403, 10040);   // Remote Port
  }

  // ===========================================================================
  // MPE720 DATA TYPE READ / WRITE (LITTLE-ENDIAN)
  // ===========================================================================

  /**
   * Read 16-bit Signed Word (MW)
   * @param {number} wordAddr - Word index (e.g. 100 for MW00100)
   */
  readMW(wordAddr) {
    const byteOffset = (wordAddr * 2) % 65536;
    return this.mView.getInt16(byteOffset, true);
  }

  /**
   * Write 16-bit Signed Word (MW)
   */
  writeMW(wordAddr, value) {
    const byteOffset = (wordAddr * 2) % 65536;
    this.mView.setInt16(byteOffset, value, true);
  }

  /**
   * Read 32-bit Signed Long Integer (ML)
   * @param {number} wordAddr - Word address start (e.g. 100 for ML00100)
   */
  readML(wordAddr) {
    const byteOffset = (wordAddr * 2) % 65536;
    return this.mView.getInt32(byteOffset, true);
  }

  /**
   * Write 32-bit Signed Long Integer (ML)
   */
  writeML(wordAddr, value) {
    const byteOffset = (wordAddr * 2) % 65536;
    this.mView.setInt32(byteOffset, value, true);
  }

  /**
   * Read 32-bit Single Precision Float (MF)
   */
  readMF(wordAddr) {
    const byteOffset = (wordAddr * 2) % 65536;
    return this.mView.getFloat32(byteOffset, true);
  }

  /**
   * Write 32-bit Single Precision Float (MF)
   */
  writeMF(wordAddr, value) {
    const byteOffset = (wordAddr * 2) % 65536;
    this.mView.setFloat32(byteOffset, value, true);
  }

  /**
   * Read 64-bit Double Precision Float (MD)
   */
  readMD(wordAddr) {
    const byteOffset = (wordAddr * 2) % 65536;
    return this.mView.getFloat64(byteOffset, true);
  }

  /**
   * Write 64-bit Double Precision Float (MD)
   */
  writeMD(wordAddr, value) {
    const byteOffset = (wordAddr * 2) % 65536;
    this.mView.setFloat64(byteOffset, value, true);
  }

  /**
   * Read Bit MB (Byte address + bit offset 0..7)
   */
  readMB(byteAddr, bitIndex = 0) {
    const b = this.mView.getUint8(byteAddr % 65536);
    return (b & (1 << (bitIndex % 8))) !== 0 ? 1 : 0;
  }

  /**
   * Write Bit MB
   */
  writeMB(byteAddr, bitIndex, val) {
    const offset = byteAddr % 65536;
    let b = this.mView.getUint8(offset);
    if (val) {
      b |= (1 << (bitIndex % 8));
    } else {
      b &= ~(1 << (bitIndex % 8));
    }
    this.mView.setUint8(offset, b);
  }

  /**
   * Read Register by MPE720 Symbol String (e.g. "MW00100", "ML00200", "MF00050", "IW00C80")
   */
  readBySymbol(symbol) {
    const sym = symbol.toUpperCase().trim();
    if (sym === 'IW00C80') return this.readIW(0);
    if (sym === 'IW00C81') return this.readIW(1);
    if (sym === 'OW00C80') return this.readOW(0);
    if (sym === 'OW00C81') return this.readOW(1);

    const match = sym.match(/^([A-Z]{2})([0-9A-F]+)$/);
    if (!match) return 0;

    const type = match[1];
    const addr = parseInt(match[2], 10);

    switch (type) {
      case 'MW': return this.readMW(addr);
      case 'ML': return this.readML(addr);
      case 'MF': return this.readMF(addr);
      case 'MD': return this.readMD(addr);
      case 'MB': return this.readMB(addr, 0);
      default: return 0;
    }
  }

  // ===========================================================================
  // LOCAL I/O REGISTER MAPPING (IW00C80 / OW00C80)
  // ===========================================================================

  readIW(index = 0) {
    // index 0 -> IW00C80 (bits 0..15), index 1 -> IW00C81 (bits 16..31)
    let word = 0;
    const start = index * 16;
    for (let i = 0; i < 16; i++) {
      if (this.DI[start + i]) {
        word |= (1 << i);
      }
    }
    return word;
  }

  readOW(index = 0) {
    let word = 0;
    const start = index * 16;
    for (let i = 0; i < 16; i++) {
      if (this.DO[start + i]) {
        word |= (1 << i);
      }
    }
    return word;
  }

  syncDIToIW() {
    this.ioView.setUint16(0x00, this.readIW(0), true);
    this.ioView.setUint16(0x02, this.readIW(1), true);
  }

  syncDOToOW() {
    this.ioView.setUint16(0x10, this.readOW(0), true);
    this.ioView.setUint16(0x12, this.readOW(1), true);
  }

  // ===========================================================================
  // DIGITAL INPUTS (DI_00..DI_31) & DIGITAL OUTPUTS (DO_00..DO_31)
  // ===========================================================================

  setDI(index, value) {
    if (index < 0 || index >= 32) return;
    const val = value ? 1 : 0;
    if (this.DI[index] !== val) {
      this.DI[index] = val;
      this.syncDIToIW();

      // Signal mapping: Control Panel switches I4..I7 (DI_20..DI_23) map to Robot IN#5..IN#8
      if (index >= 20 && index <= 23) {
        const inNum = index - 20 + 5;
        this.robotIN[inNum] = val;
      }

      this.notifyChange();
    }
  }

  getDI(index) {
    if (index < 0 || index >= 32) return 0;
    return this.DI[index];
  }

  setDO(index, value) {
    if (index < 0 || index >= 32) return;
    const val = value ? 1 : 0;
    if (this.DO[index] !== val) {
      this.DO[index] = val;
      this.syncDOToOW();

      // Mapping: DO_01 -> Robot IN#1 (Pack ready at pickup station)
      if (index === 1) {
        this.robotIN[1] = val;
      }
      // Mapping: DO_02 -> Robot IN#2 (Box ready status)
      if (index === 2) {
        this.robotIN[2] = val;
      }

      this.notifyChange();
    }
  }

  getDO(index) {
    if (index < 0 || index >= 32) return 0;
    return this.DO[index];
  }

  // ===========================================================================
  // ROBOT SIGNALS & PULSE HANDLER
  // ===========================================================================

  setRobotOT(otNum, value) {
    if (otNum < 1 || otNum > 8) return;
    const val = value ? 1 : 0;
    this.robotOT[otNum] = val;

    // Mapping: OT#5..OT#8 -> DO_20..DO_23 (Control panel LEDs Q4..Q7)
    if (otNum >= 5 && otNum <= 8) {
      this.setDO(15 + otNum, val);
    }

    // Mapping: OT#3 pulse -> DI_12 (Robot Arm finished drop in carton box)
    if (otNum === 3 && val === 1) {
      this.triggerOT3Pulse();
    }

    this.notifyChange();
  }

  triggerOT3Pulse() {
    this.setDI(12, 1);
    this.setDO(24, 1); // DO_24 = Q0 on I/O box lights up

    setTimeout(() => {
      this.setDI(12, 0);
      this.setRobotOT(3, 0);
      setTimeout(() => {
        this.setDO(24, 0);
      }, 500);
    }, 100); // 100ms standard pulse
  }

  setRobotMode(modeIndex) {
    // 0 = TEACH, 1 = PLAY, 2 = REMOTE
    this.robotMode = modeIndex;
    this.notifyChange();
  }

  getRobotModeName() {
    switch (this.robotMode) {
      case 0: return 'TEACH';
      case 1: return 'PLAY';
      case 2: return 'REMOTE';
      default: return 'UNKNOWN';
    }
  }

  setEmergencyStop(pressed) {
    this.emergencyStop = pressed;
    if (pressed) {
      // Emergency: Immediate cut off of all drive outputs
      this.setDO(0, 0); // Stop Conveyor 1 & 2
      this.setDO(3, 0); // Stop Conveyor 3
      this.setDO(5, 0); // Vacuum OFF
      this.plcState = 'IDLE';
    }
    this.notifyChange();
  }

  // ===========================================================================
  // BUILT-IN DEFAULT PLC LADDER LOGIC (Bài 2 Sequencing)
  // ===========================================================================

  /**
   * Executes one cycle of PLC scan logic
   * @param {number} delta - Frame delta time in seconds
   * @param {Object} context - { runRobotPickCycle: Function }
   */
  updatePLCLogic(delta, context = {}) {
    if (this.emergencyStop) return;

    // 1. Process START / STOP push buttons
    // DI_00: START Button
    if (this.getDI(0) === 1 && this.plcState === 'IDLE') {
      this.plcState = 'RUNNING_CONVEYOR';
      this.setDO(0, 1); // Run product conveyor (J1000 Inverter S1)
    }

    // DI_01: STOP Button
    if (this.getDI(1) === 1 && this.plcState !== 'IDLE') {
      this.plcState = 'IDLE';
      this.setDO(0, 0);
      this.setDO(3, 0);
      this.setDO(1, 0);
    }

    if (!this.builtinPLCOn) return;

    // 2. Automated State Machine
    switch (this.plcState) {
      case 'RUNNING_CONVEYOR':
        // Keep Infeed & Collation Conveyors (DO_00) and Rear Box Conveyor (DO_03) running continuously
        // so Delta Robot can keep picking bars on Conveyor 1 while 6-DOF Arm packs at DI_11!
        this.setDO(0, 1);
        this.setDO(3, 1);
        // Check if 4-pack has arrived at sensor DI_11
        if (this.getDI(11) === 1) {
          // Send IN#1 to Robot Arm (DO_01 = 1) without stopping the conveyor!
          this.setDO(1, 1);
          this.plcState = 'WAITING_ROBOT_PICK';

          // Trigger automated robot pick cycle if available
          if (context.runRobotPickCycle && !context.isRobotBusy) {
            this.plcState = 'ROBOT_PACKING';
            context.runRobotPickCycle().then(() => {
              // Pulse OT#3 is triggered by robot when placed
            });
          }
        }
        break;

      case 'ROBOT_PACKING':
        // Wait for DI_12 (OT#3 pulse from Robot Arm)
        if (this.getDI(12) === 1) {
          this.setDO(1, 0); // Clear pack ready flag
          this.plcBoxClusterCount++;

          if (this.plcBoxClusterCount >= 2) {
            // Box is full (2 layers packed) -> Discharge box
            this.plcBoxClusterCount = 0;
            this.plcState = 'DISCHARGING_BOX';
            this.setDO(3, 1); // Run Conveyor 3 to push box out
          } else {
            // Box not full yet, restart conveyor for next pack
            this.plcState = 'RUNNING_CONVEYOR';
            this.setDO(0, 1);
          }
        }
        break;

      case 'DISCHARGING_BOX':
        // Wait until new empty box arrives at stopper (DI_13 = 1)
        if (this.getDI(13) === 1) {
          this.setDO(3, 0); // Stop Conveyor 3
          // Resume infeed conveyor
          this.setDO(0, 1);
          this.plcState = 'RUNNING_CONVEYOR';
        }
        break;

      default:
        break;
    }
  }
}
