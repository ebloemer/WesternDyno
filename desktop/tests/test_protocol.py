import struct,unittest,zlib
from protocol import *
from settings import MachineParameters,SetupProfile

def telemetry_packet():
    body=struct.pack(TELEMETRY_FORMAT_NO_CRC,PACKET_MAGIC,PACKET_VERSION,int(SystemState.DISARMED),int(DynoMode.MANUAL),0,int(TelemetryFlag.ACTIVE_SETUP_VALID),0,0,7,1795.,1800.,2000.,1200.,1400.,35.,40.,25.,60.,50.)
    return body+struct.pack("<I",zlib.crc32(body)&0xffffffff)

class ProtocolTests(unittest.TestCase):
    def test_sizes_and_crc(self):
        packet=Command(sequence=42,arm=True,mode=DynoMode.ENGINE_RPM,manual_throttle_pct=100,manual_throttle_override=True,target_engine_rpm=3800).pack()
        self.assertEqual((len(packet),COMMAND_SIZE,TELEMETRY_SIZE,CONFIG_SIZE),(36,36,64,60));self.assertEqual(struct.unpack_from("<I",packet,-4)[0],zlib.crc32(packet[:-4]))
    def test_telemetry(self):
        value=Telemetry.unpack(telemetry_packet());self.assertEqual(value.engine_sensor_rpm,1795);self.assertEqual(value.engine_rpm,1800);self.assertAlmostEqual(value.power_kw,35*1200/9549.296596)
    def test_corruption(self):
        packet=bytearray(telemetry_packet());packet[20]^=1
        with self.assertRaises(ProtocolError):Telemetry.unpack(bytes(packet))
    def test_config_kinds(self):
        packets=[Config.machine(MachineParameters()),Config.pid(0,MachineParameters().pid_banks["pressure_torque"]),Config.active_setup(SetupProfile())]
        for config in packets:self.assertEqual(Config.unpack(config.pack()).kind,config.kind)
    def test_reversed_servo_endpoints_are_transmitted_without_reverse_flag(self):
        setup=SetupProfile(throttle_closed_us=1800,throttle_full_us=900)
        config=Config.active_setup(setup)
        self.assertEqual(config.values[3:5],(1800,900))
