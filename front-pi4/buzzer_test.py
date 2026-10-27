import argparse, time
from buzzer import GpioBuzzer
ap = argparse.ArgumentParser()
ap.add_argument('--gpio', type=int, default=18)
a = ap.parse_args()
bz = GpioBuzzer(a.gpio)
try:
    print('1) 1초 켜기'); bz.on(); time.sleep(1)
    print('2) 끄기'); bz.off(); time.sleep(1)
    print('3) 주의 패턴 3초'); bz.set_level(1); time.sleep(3)
    print('4) 위험 패턴 3초'); bz.set_level(2); time.sleep(3)
    print('5) 끝'); bz.off()
except KeyboardInterrupt:
    pass
finally:
    bz.close()
