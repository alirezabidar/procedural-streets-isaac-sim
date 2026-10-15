"""Cook parameter variations and verify the HDA is deterministic and portable."""
import os,json,hashlib
from pathlib import Path
os.environ['PATH']=os.pathsep.join(p for p in os.environ['PATH'].split(os.pathsep) if 'WindowsApps' not in p)
import hou
ROOT=Path(__file__).resolve().parents[1]
hou.hipFile.load(str(ROOT/'houdini/procedural_driving.hip'),suppress_save_prompt=True)
a=hou.node('/obj/procedural_street_district/street_district')
defaults={p.name():p.eval() for p in a.parms() if p.name() in ['blocks_x','blocks_z','pitch_x','pitch_z','road_width','sidewalk','seed','trees','lamps','crosswalks','footprint']}
def sample(name,changes):
    a.setParms(defaults);a.setParms(changes);g=a.geometry()
    digest=hashlib.sha256(repr([(tuple(p.position())) for p in g.points()]).encode()).hexdigest()
    assert not a.errors(),a.errors()
    return dict(name=name,parameters=changes,points=len(g.points()),primitives=len(g.prims()),bounds=list(g.boundingBox().minvec())+list(g.boundingBox().maxvec()),geometry_hash=digest)
rows=[sample('default',{}),sample('alternate_seed',{'seed':38}),sample('default_repeat',{}),
      sample('expanded',{'blocks_x':4,'blocks_z':3,'seed':38}),sample('tight_blocks',{'pitch_x':36,'pitch_z':34,'road_width':14,'sidewalk':3.5,'footprint':1.05}),
      sample('details_off',{'trees':0,'lamps':0,'crosswalks':0})]
assert rows[0]['geometry_hash']==rows[2]['geometry_hash']
assert rows[0]['geometry_hash']!=rows[1]['geometry_hash']
assert rows[3]['points']>rows[0]['points']>rows[5]['points']
(ROOT/'results').mkdir(exist_ok=True)
(ROOT/'results/hda-validation.json').write_text(json.dumps({'passed':True,'cases':rows},indent=2))
print(json.dumps({'passed':True,'cases':len(rows)}))
