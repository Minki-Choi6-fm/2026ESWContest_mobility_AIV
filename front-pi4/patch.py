p='front_node.py'; L=open(p).read().split('\n')
if any('buzzer' in x for x in L):
    print('이미 부저 코드가 있음'); raise SystemExit
out=[]
for x in L:
    if x.strip()=='args = ap.parse_args()':
        out += ["    ap.add_argument('--buzzer-gpio', type=int, default=None)", x,
                "    buzzer = None", "    if args.buzzer_gpio is not None:",
                "        from buzzer import GpioBuzzer", "        buzzer = GpioBuzzer(args.buzzer_gpio)"]
    elif 'stm32.send_warn(level, src, dist)' in x:
        out += [x, "            if buzzer is not None:", "                buzzer.set_level(level)"]
    elif x.strip()=='finally:' and len(x)-len(x.lstrip())==4:
        out += [x, "        if buzzer is not None:", "            buzzer.close()"]
    else:
        out.append(x)
open(p,'w').write('\n'.join(out)); print('부저 코드 추가 완료')
