import json,sys
def load(d):
    try: return {x['board']:x for x in json.load(open(d+'/results.json'))}
    except Exception: return {}
A=load(sys.argv[1]); S=load(sys.argv[2]) if len(sys.argv)>2 else {}; I=load(sys.argv[3]) if len(sys.argv)>3 else {}
def cell(x):
    if x is None: return '-'
    s=f"{x['routed']}"
    if x.get('places'): s+=f" p{x['places']} {x['worst_um']}"
    return s
cases=[]
for b in A:
    if A[b]['case'] not in cases: cases.append(A[b]['case'])
print('case | need / avail(tight) mm | ref items L/A | loose | tight20 | tight | exact | short || settle300 t20 | t || it1600 t20 | t')
for c in cases:
    g=lambda f,D=A: D.get(f'{c}.{f}')
    t=A[f'{c}.tight']; ri=t['reference_items']
    print(f"{c} | {t['need_mm']:.3f} / {t['available_mm']:.3f} | {ri['lines']}/{ri['arcs']} | {cell(g('loose'))} | {cell(g('tight20'))} | {cell(g('tight'))} | {cell(g('exact'))} | {cell(g('short'))} || {cell(g('tight20',S))} | {cell(g('tight',S))} || {cell(g('tight20',I))} | {cell(g('tight',I))}")
