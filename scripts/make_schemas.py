"""Build the checked-in JSON Schemas. Run after changing format definitions here."""
import json
from pathlib import Path

def obj(properties, required=(), **kw):
    return {"type":"object", "properties":properties,"required":list(required),"additionalProperties":False,**kw}
def arr(items, **kw): return {"type":"array","items":items,**kw}
def ref(name): return {"$ref":"#/$defs/"+name}
number={"type":"number"}
positive={"type":"number","exclusiveMinimum":0}
nonnegative={"type":"number","minimum":0}
string={"type":"string","minLength":1}
boolean={"type":"boolean"}
v3=arr(number,minItems=3,maxItems=3)
kv={"type":"object"}
pose=obj({"position_mm":v3,"rotation_deg":v3})
frame=obj({"position_mm":v3,"axis":v3,"rotation_deg":v3})
endpoint=obj({"part":string,"port":string,"end":{"enum":["start","end"]},"at_mm":nonnegative,"frame":frame},["part"])
shape=obj({"type":{"enum":["box","tube","cylinder","capsule","sphere","mesh","extrusion"]},"size_mm":v3,"length_mm":positive,"diameter_mm":positive,"wall_mm":positive,"radius_mm":positive,"position_mm":v3,"rotation_deg":v3,"axis":v3,"file":string,"scale":positive,"color":string,"collision":{"enum":["convex","static_mesh"]},"collision_geometry":arr(kv)},["type"])
body=obj({"kind":{"enum":["member","connector","panel","wheel","load","human","rigid","belt","chain"]},"geometry":arr(shape,minItems=1),"ports":kv,"section":kv,"material":string,"mass_kg":nonnegative,"mass_per_m_kg":positive,"center_of_mass_mm":v3,"inertia_kg_m2":v3,"color":string,"friction":nonnegative,"restitution":{"type":"number","minimum":0,"maximum":1},"length_mm":positive,"strength":kv,"source":kv,"name":string})
part=obj({"id":string,"catalog":string,"parameters":kv,"pose":pose,"body":body,"mass_kg":nonnegative,"color":string,"friction":nonnegative,"restitution":{"type":"number","minimum":0,"maximum":1},"center_of_mass_mm":v3,"inertia_kg_m2":v3,"label":string,"metadata":kv},["id"],anyOf=[{"required":["catalog"]},{"required":["body"]}])
limits=obj({"slide_mm":arr(number,minItems=2,maxItems=2),"angle_deg":arr(number,minItems=2,maxItems=2),"rotation_deg":arr(arr(number,minItems=2,maxItems=2),minItems=3,maxItems=3)})
motor=obj({"mode":{"enum":["position","velocity"]},"target":number,"max_force_n":positive,"max_torque_nm":positive,"kp":nonnegative,"kd":nonnegative,"schedule":arr(obj({"time_s":nonnegative,"target":number},["time_s","target"]))})
joint=obj({"id":string,"type":{"enum":["socket","fixed","revolute","prismatic","cylindrical","spherical","distance"]},"a":endpoint,"b":endpoint,"locked":boolean,"insertion_mm":nonnegative,"fit_tolerance_mm":{"type":"number","minimum":.01,"maximum":20},"limits":limits,"motor":motor,"damping":nonnegative,"friction":nonnegative,"rest_length_mm":nonnegative,"tension_only":boolean,"max_force_n":positive,"max_torque_nm":positive,"break_force_n":positive,"break_torque_nm":positive,"torque_nm":nonnegative,"assembly":{"enum":["slide","radial","bolt","glue","hook"]},"metadata":kv},["id","type","a","b"])
anchor=obj({"part":string,"label":string,"surface":{"enum":["floor","wall","ceiling","fixture"]},"dofs":arr({"enum":["x","y","z","rx","ry","rz"]},uniqueItems=True),"position_mm":v3,"capacity_n":positive},["part"])
load=obj({"part":string,"force_n":v3,"moment_nm":v3,"at_mm":nonnegative,"point_mm":v3,"start_s":nonnegative,"end_s":nonnegative},["part"])
coordinates=obj({"slide_mm":number,"angle_deg":number,"twist_deg":number,"rotation_deg":v3})
test=obj({"id":string,"type":{"enum":["reach","seat","clearance"]},"human":string,"target_mm":v3,"hand":{"enum":["left","right"]},"seat":string,"clearance_mm":nonnegative,"expect":boolean},["id","type"])
design=obj({"format":{"const":"pipesim/1"},"name":string,"description":{"type":"string"},"units":{"const":"mm-kg-s-N-deg"},"libraries":arr(string),"definitions":{"type":"object","additionalProperties":body},"parts":arr(part),"joints":arr(joint),"objects":arr(obj({"id":string,"template":string,"parameters":kv,"pose":pose},["id","template"])),"anchors":arr(anchor),"loads":arr(load),"environment":obj({"gravity_m_s2":v3,"ground":boolean,"ground_z_mm":number,"friction":nonnegative}),"state":obj({"joints":{"type":"object","additionalProperties":coordinates},"time_s":nonnegative}),"animation":obj({"duration_s":positive,"fps":positive,"tracks":arr(obj({"joint":string,"coordinate":{"enum":["slide_mm","angle_deg","twist_deg"]},"keyframes":arr(obj({"time_s":nonnegative,"value":number},["time_s","value"]),minItems=1)},["joint","coordinate","keyframes"]))}),"drives":arr(obj({"id":string,"type":{"enum":["gt2","gear","rack"]},"driver":string,"follower":string,"pitch_mm":positive,"teeth":{"type":"integer","minimum":1},"ratio":number,"sign":{"enum":[-1,1]},"efficiency":{"type":"number","exclusiveMinimum":0,"maximum":1},"max_force_n":positive,"stiffness_n_m":positive,"damping_ns_m":nonnegative,"belt_length_mm":positive,"belt_width_mm":positive,"route_mm":arr(v3)},["id","type","driver","follower"])),"tests":arr(test),"build":obj({"kerf_mm":nonnegative,"end_trim_mm":nonnegative,"stock_lengths_mm":arr(positive),"require_stability":boolean,"allow_temporary_supports":boolean,"search_limit":{"type":"integer","minimum":1},"sequence":arr(string),"fixtures":arr(obj({"part":string,"description":string},["part","description"]))}),"results":kv,"build_plan":kv,"metadata":kv},["format","units","parts"])
library=obj({"format":{"const":"pipesim-library/1"},"name":string,"description":{"type":"string"},"source":kv,"materials":kv,"parts":kv,"objects":kv},["format","name"])
# Definitions are parameterised library templates; numeric geometry is checked after substitution.
design['properties']['definitions']={'type':'object','additionalProperties':{'type':'object'}}
design['properties']['drives']['items']['properties'].update({'offset_mm':number,'offset_deg':number})
motor['properties']['rotation_deg']=v3
# A regrouped object keeps its edited local parts and joints as authoritative
# components. Parameters describe the editable human controls around that pose.
components=obj({'parts':arr(ref('part'),minItems=1),'joints':arr(ref('joint')),'anchors':arr(anchor),'parameter_reference':kv},['parts'])
instance=obj({'id':string,'label':string,'template':string,'parameters':kv,'pose':pose,'components':components,'layout_mode':{'enum':['rigid','posable']}},['id','template'])
expanded=obj({'instance':instance,'reference_part':string,'reference_pose':pose,'index':{'type':'integer','minimum':0}},['instance','reference_part','reference_pose'])
design['$defs']={'part':part,'joint':joint}
design['properties']['objects']=arr(instance)
design['properties']['expanded_objects']=arr(expanded)
for name,schema in (("design",design),("library",library)):
    schema={"$schema":"https://json-schema.org/draft/2020-12/schema","$id":f"https://pipesim.local/schema/{name}/1",**schema}
    path=Path(__file__).resolve().parents[1]/"pipesim"/"data"/"schemas"/f"{name}.schema.json"
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(schema,indent=2)+"\n",encoding="utf-8")
