/**
 * YRC1000EthernetServer.js
 * 
 * High-Speed Ethernet Server Emulation (UDP Port 10040) for Yaskawa YRC1000 / YRC1000micro
 * 
 * Features:
 * 1. Network Topology:
 *    - Client: MP3300 218IFD Module (IP: 192.168.1.1, Port: 10001, Subnet: 255.255.255.0)
 *    - Server: YRC1000micro Ethernet Server (IP: 192.168.1.11, Port: 10040, Protocol: UDP, Code: BIN)
 * 2. Standard 32-Byte "YERC" Frame Header Builder & Parser:
 *    - Identifier: 0x43524559 (ASCII: "YERC")
 *    - HeaderSize: 0x0020 (32 bytes)
 *    - DataSize: payload size
 *    - Reserve1: 0x03, PacketType: 0x01 (Robot control)
 *    - Ack: 0x00 (Request), 0x01 (Answer)
 *    - Reserve2: "99999999" (8 bytes ASCII '9')
 * 3. High-Speed Ethernet Commands:
 *    - 0x0083: Servo ON / OFF (Data: 1 = ON, 2 = OFF)
 *    - 0x0086: Start JOB (Data: 1 = Start)
 *    - 0x0072: Read Status (Running, Servo ON, Mode, Alarm)
 *    - 0x0075: Read Position (Cartesian / Pulse)
 *    - 0x008A: Move Cartesian / Pulse
 * 4. Hardware Safety Check:
 *    - Requires Mode Switch == REMOTE (2). If in TEACH (0) or PLAY (1), returns Error Status 0x1F.
 * 5. MPE720 PLC Function Block MSG_SNDE:
 *    - Execute (GB000100), Busy (GB000102), Complete (GB000103), Error (GB000104)
 * 6. Real-Time Packet Sniffer & Hex Monitor.
 */

export class YRC1000EthernetServer {
  constructor(hardwareBus, robotArm, options = {}) {
    this.hardwareBus = hardwareBus;
    this.robotArm = robotArm;

    // Network Configuration
    this.config = {
      clientIP: '192.168.1.1',
      clientPort: 10001,
      serverIP: '192.168.1.11',
      serverPort: 10040,
      subnetMask: '255.255.255.0',
      protocol: 'UDP',
      code: 'BIN'
    };

    // Server Internal State
    this.servoOn = false;
    this.jobRunning = false;
    this.alarmActive = false;
    this.alarmCode = 0;

    // Packet Sequence Counter (0..255)
    this.requestCounter = 1;

    // Packet Sniffer History (Stores last 50 transactions for UI display)
    this.packetHistory = [];
    this.historyListeners = new Set();
  }

  subscribeHistory(fn) {
    this.historyListeners.add(fn);
    return () => this.historyListeners.delete(fn);
  }

  notifyHistory(item) {
    this.packetHistory.unshift(item);
    if (this.packetHistory.length > 50) {
      this.packetHistory.pop();
    }
    for (const fn of this.historyListeners) {
      fn(item, this.packetHistory);
    }
  }

  // ===========================================================================
  // 32-BYTE YERC FRAME BUILDER (LITTLE-ENDIAN BINARY FORMAT)
  // ===========================================================================

  /**
   * Build 32-Byte YERC Header Buffer
   * @param {number} dataSize - Size of sub-header + payload (bytes)
   * @param {number} reqId - Request ID
   * @param {number} ack - 0x00 for Request, 0x01 for Answer
   * @returns {Uint8Array} 32-byte header
   */
  buildYERCHeader(dataSize, reqId = 1, ack = 0x00) {
    const buf = new ArrayBuffer(32);
    const view = new DataView(buf);
    const u8 = new Uint8Array(buf);

    // 0..3: Identifier "YERC" (ASCII: 0x59, 0x45, 0x52, 0x43)
    u8[0] = 0x59; // 'Y'
    u8[1] = 0x45; // 'E'
    u8[2] = 0x52; // 'R'
    u8[3] = 0x43; // 'C'

    // 4..5: Header Size (uint16 = 32)
    view.setUint16(4, 32, true);

    // 6..7: Data Size (uint16)
    view.setUint16(6, dataSize, true);

    // 8: Reserve 1 (0x03)
    u8[8] = 0x03;

    // 9: Packet Type (0x01 = Robot Control)
    u8[9] = 0x01;

    // 10: ACK (0x00 = Request, 0x01 = Answer)
    u8[10] = ack;

    // 11: Request ID
    u8[11] = reqId & 0xff;

    // 12..15: Block No (uint32 = 0)
    view.setUint32(12, 0, true);

    // 16..23: Reserve 2 ("99999999" -> 8 bytes of ASCII '9' = 0x39)
    for (let i = 16; i < 24; i++) {
      u8[i] = 0x39;
    }

    // 24..31: Reserved / Zero padding
    for (let i = 24; i < 32; i++) {
      u8[i] = 0x00;
    }

    return u8;
  }

  /**
   * Builds a complete UDP Request Packet
   */
  buildRequestPacket(commandNo, instance, attribute, service, dataBytes = []) {
    const payloadSize = 8 + dataBytes.length; // 8 bytes command sub-header + data
    const header = this.buildYERCHeader(payloadSize, this.requestCounter++, 0x00);

    const fullPacket = new Uint8Array(32 + payloadSize);
    fullPacket.set(header, 0);

    const view = new DataView(fullPacket.buffer);

    // Offset 32: Command No (uint16)
    view.setUint16(32, commandNo, true);
    // Offset 34: Instance (uint16)
    view.setUint16(34, instance, true);
    // Offset 36: Attribute (uint8)
    fullPacket[36] = attribute;
    // Offset 37: Service (uint8)
    fullPacket[37] = service;
    // Offset 38..39: Padding in request
    fullPacket[38] = 0x00;
    fullPacket[39] = 0x00;

    // Offset 40..: Data bytes
    for (let i = 0; i < dataBytes.length; i++) {
      fullPacket[40 + i] = dataBytes[i];
    }

    return fullPacket;
  }

  // ===========================================================================
  // SERVER PACKET DISPATCHER & COMMAND EXECUTION
  // ===========================================================================

  /**
   * Simulates receiving a UDP packet at YRC1000micro (192.168.1.11:10040)
   * Parses the request, validates mode switch, executes command, and returns Answer packet.
   * @param {Uint8Array} requestPacket - Raw binary packet
   * @returns {Object} { answerPacket: Uint8Array, error: boolean, message: string }
   */
  processUDPPacket(requestPacket) {
    if (requestPacket.length < 40) {
      return { error: true, message: 'Gói tin không đủ 32-byte Header và Sub-Header' };
    }

    const view = new DataView(requestPacket.buffer);

    // Validate "YERC" Identifier
    const id0 = requestPacket[0];
    const id1 = requestPacket[1];
    const id2 = requestPacket[2];
    const id3 = requestPacket[3];
    if (id0 !== 0x59 || id1 !== 0x45 || id2 !== 0x52 || id3 !== 0x43) {
      return { error: true, message: 'Mã định danh không hợp lệ (Không phải YERC)' };
    }

    const reqId = requestPacket[11];
    const commandNo = view.getUint16(32, true);
    const instance = view.getUint16(34, true);
    const attribute = requestPacket[36];
    const service = requestPacket[37];

    // Payload data begins at offset 40
    const payloadLength = requestPacket.length - 40;
    let dataVal = 0;
    if (payloadLength >= 4) {
      dataVal = view.getUint32(40, true);
    }

    // Hardware Safety Check: Must be in REMOTE mode for robot motion and servo control
    const currentMode = this.hardwareBus ? this.hardwareBus.robotMode : 2;
    let status = 0x00; // 0x00 = Success
    let addedStatus = 0x0000;
    let answerData = [];
    let logMsg = '';

    // Chaos Engineering: Check if Packet Drop is simulated
    if (this.simulatePacketDrop) {
      status = 0x1F;
      addedStatus = 0x4040;
      logMsg = `[MẤT KẾT NỐI MẠNG] Gói tin UDP 10040 bị drop (Chaos Packet Loss Simulation). YRC1000 không phản hồi!`;
    } else {
      // Check if command requires REMOTE mode
      const isControlCommand = (commandNo === 0x0083 || commandNo === 0x0086 || commandNo === 0x008A);
      if (isControlCommand && currentMode !== 2) {
        // 0 = TEACH, 1 = PLAY -> Reject remote Ethernet control!
        status = 0x1F; // Vendor Error Code
        addedStatus = 0x2030; // "Cannot execute remote command when Mode Switch is not REMOTE"
        logMsg = `[TỪ CHỐI LỆNH] Khóa Mode Switch đang ở [${this.hardwareBus.getRobotModeName()}], không cho phép điều khiển qua Ethernet Server (Cần gạt sang REMOTE)!`;
      } else {
      // Execute command according to YRC1000 specification
      switch (commandNo) {
        case 0x0083: // Servo ON / OFF Control
          if (dataVal === 1) {
            this.servoOn = true;
            logMsg = `[SERVO ON THÀNH CÔNG] YRC1000 đã cấp nguồn động lực Servo cho Robot Arm 6 bậc.`;
          } else if (dataVal === 2) {
            this.servoOn = false;
            this.jobRunning = false;
            logMsg = `[SERVO OFF THÀNH CÔNG] YRC1000 đã ngắt nguồn động lực Servo.`;
          }
          break;

        case 0x0086: // Start JOB
          if (dataVal === 1) {
            if (!this.servoOn) {
              status = 0x1F;
              addedStatus = 0x2040;
              logMsg = `[LỖI START JOB] Không thể chạy JOB khi Servo đang OFF!`;
            } else {
              this.jobRunning = true;
              logMsg = `[START JOB THÀNH CÔNG] Bắt đầu thực thi chương trình gia công / đóng gói.`;
            }
          }
          break;

        case 0x0072: // Read Status
          {
            // Returns 8-byte status bitmap
            const statusBuf = new ArrayBuffer(8);
            const sView = new DataView(statusBuf);
            // Byte 0: Mode bits (bit 5 = TEACH, bit 6 = PLAY, bit 7 = REMOTE)
            let b0 = 0;
            if (currentMode === 0) b0 |= (1 << 5);
            if (currentMode === 1) b0 |= (1 << 6);
            if (currentMode === 2) b0 |= (1 << 7);
            if (this.jobRunning) b0 |= (1 << 3); // Running bit
            sView.setUint8(0, b0);

            // Byte 1: Servo ON bit (bit 0)
            let b1 = this.servoOn ? 1 : 0;
            sView.setUint8(1, b1);

            answerData = Array.from(new Uint8Array(statusBuf));
            logMsg = `[ĐỌC TRẠNG THÁI 0x72] Servo: ${this.servoOn ? 'ON' : 'OFF'} | Mode: ${this.hardwareBus.getRobotModeName()} | Running: ${this.jobRunning}`;
          }
          break;

        case 0x0075: // Read Position
          {
            // Returns 32-byte Cartesian Position: Data type (1), Coordinate frame (1), X, Y, Z, Rx, Ry, Rz
            const posBuf = new ArrayBuffer(32);
            const pView = new DataView(posBuf);
            pView.setUint32(0, 16, true); // Cartesian data type
            pView.setUint32(4, 0, true);  // Base coordinate frame

            const tcp = this.robotArm ? this.robotArm.getTCPWorldPosition() : { x: 0, y: 0, z: 0 };
            pView.setInt32(8, Math.round(tcp.x * 1000000), true); // 0.001 mm
            pView.setInt32(12, Math.round(tcp.y * 1000000), true);
            pView.setInt32(16, Math.round(tcp.z * 1000000), true);
            answerData = Array.from(new Uint8Array(posBuf));
            logMsg = `[ĐỌC TỌA ĐỘ 0x75] TCP X=${tcp.x.toFixed(3)}, Y=${tcp.y.toFixed(3)}, Z=${tcp.z.toFixed(3)}`;
          }
          break;

        default:
          status = 0x00;
          logMsg = `[LỆNH 0x${commandNo.toString(16).toUpperCase()}] Đã xử lý lệnh thành công.`;
          break;
      }
    }
  }

    // Build Answer Packet (ACK = 0x01)
    const ansPayloadSize = 8 + answerData.length;
    const ansHeader = this.buildYERCHeader(ansPayloadSize, reqId, 0x01);
    const answerPacket = new Uint8Array(32 + ansPayloadSize);
    answerPacket.set(ansHeader, 0);

    const ansView = new DataView(answerPacket.buffer);
    ansView.setUint16(32, commandNo, true);
    ansView.setUint16(34, instance, true);
    answerPacket[36] = attribute;
    answerPacket[37] = service | 0x80; // Answer service flag
    answerPacket[38] = status;
    answerPacket[39] = (addedStatus & 0xff);

    for (let i = 0; i < answerData.length; i++) {
      answerPacket[40 + i] = answerData[i];
    }

    const transaction = {
      timestamp: new Date().toLocaleTimeString(),
      reqId,
      commandNo: `0x${commandNo.toString(16).toUpperCase().padStart(4, '0')}`,
      status: status === 0x00 ? 'SUCCESS (0x00)' : `ERROR (0x${status.toString(16).toUpperCase()})`,
      addedStatus: addedStatus !== 0 ? `0x${addedStatus.toString(16).toUpperCase()}` : '0x00',
      requestHex: this.formatHexDump(requestPacket),
      answerHex: this.formatHexDump(answerPacket),
      logMsg,
      isError: status !== 0x00
    };

    this.notifyHistory(transaction);

    return {
      answerPacket,
      error: status !== 0x00,
      status,
      addedStatus,
      message: logMsg
    };
  }

  // ===========================================================================
  // MPE720 FUNCTION BLOCK: MSG_SNDE SIMULATION
  // ===========================================================================

  /**
   * Executes MPE720 Ladder Function Block: MSG_SNDE
   * Sends command from MP3300 (192.168.1.1:10001) to YRC1000 (192.168.1.11:10040)
   * 
   * @param {string} cmdType - 'SERVO_ON', 'SERVO_OFF', 'START_JOB', 'READ_STATUS'
   * @param {Array} [dataBytes=[]] - Additional payload data
   * @returns {Promise<Object>} Execution result
   */
  async sendMSGSNDE(cmdType, dataBytes = []) {
    // 1. Set Busy Flag: GB000102 = 1, Complete: GB000103 = 0, Error: GB000104 = 0
    this.hardwareBus.writeMB(102, 0, 1); // Busy
    this.hardwareBus.writeMB(103, 0, 0); // Complete
    this.hardwareBus.writeMB(104, 0, 0); // Error

    let reqPacket;
    switch (cmdType) {
      case 'SERVO_ON':
        reqPacket = this.buildRequestPacket(0x0083, 0x0002, 0x01, 0x10, [1, 0, 0, 0]);
        break;
      case 'SERVO_OFF':
        reqPacket = this.buildRequestPacket(0x0083, 0x0002, 0x01, 0x10, [2, 0, 0, 0]);
        break;
      case 'START_JOB':
        reqPacket = this.buildRequestPacket(0x0086, 0x0001, 0x01, 0x10, [1, 0, 0, 0]);
        break;
      case 'READ_STATUS':
        reqPacket = this.buildRequestPacket(0x0072, 0x0001, 0x01, 0x0E, []);
        break;
      case 'READ_POS':
        reqPacket = this.buildRequestPacket(0x0075, 0x0001, 0x01, 0x0E, [16, 0, 0, 0]);
        break;
      default:
        reqPacket = this.buildRequestPacket(0x0072, 0x0001, 0x01, 0x0E, []);
        break;
    }

    // Simulate high-speed network transmission latency (20ms)
    await new Promise(res => setTimeout(res, 20));

    const result = this.processUDPPacket(reqPacket);

    // Update PLC Completion / Error flags
    this.hardwareBus.writeMB(102, 0, 0); // Busy = 0
    if (result.error) {
      this.hardwareBus.writeMB(104, 0, 1); // Error = 1
    } else {
      this.hardwareBus.writeMB(103, 0, 1); // Complete = 1
    }

    return result;
  }

  // ===========================================================================
  // HEX FRAME MONITOR FORMATTER
  // ===========================================================================

  /**
   * Formats raw binary packet into standard Hex Dump with offsets and ASCII representation
   */
  formatHexDump(uint8Arr) {
    const lines = [];
    const len = uint8Arr.length;

    for (let i = 0; i < len; i += 16) {
      const chunk = uint8Arr.subarray(i, Math.min(i + 16, len));
      const offsetStr = i.toString(16).toUpperCase().padStart(4, '0');

      let hexParts = [];
      let asciiParts = [];

      for (let j = 0; j < 16; j++) {
        if (j < chunk.length) {
          const b = chunk[j];
          hexParts.push(b.toString(16).toUpperCase().padStart(2, '0'));
          // Printable ASCII
          if (b >= 32 && b <= 126) {
            asciiParts.push(String.fromCharCode(b));
          } else {
            asciiParts.push('.');
          }
        } else {
          hexParts.push('  ');
          asciiParts.push(' ');
        }
      }

      // Group into 8-byte halves for readability
      const hexGroup1 = hexParts.slice(0, 8).join(' ');
      const hexGroup2 = hexParts.slice(8, 16).join(' ');
      const asciiStr = asciiParts.join('');

      lines.push(`${offsetStr}  ${hexGroup1}  ${hexGroup2}  |${asciiStr}|`);
    }

    return lines.join('\n');
  }
}
