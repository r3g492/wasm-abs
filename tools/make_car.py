# Generates the sedan used by the simulator and exports it as glTF.
# Run inside Blender (any 4.x/5.x):  exec(open("<repo>/tools/make_car.py").read())
# or headless:  blender -b -P tools/make_car.py
#
# Blender axes: X forward, Y left, Z up. Origin = ground point under the CG.
# glTF export turns this into X forward, Y up, Z right, which is what src/main.c draws in.
# Must match src/sim.h: CAR_A 1.15, CAR_B 1.45, CAR_TRACK 1.55, CAR_R 0.31.

import bpy, bmesh, os, math
from mathutils import Vector

REPO = os.environ.get("ABS_REPO", "/Users/g23k/Projects/wasm-abs")
OUT = os.path.join(REPO, "assets")
A, B, TRACK, R = 1.15, 1.45, 1.55, 0.31
TIRE_W = 0.215

# ---------------------------------------------------------------------------
# Scene + materials
# ---------------------------------------------------------------------------
scene = bpy.data.scenes.get("ABSCar") or bpy.data.scenes.new("ABSCar")
for o in list(scene.collection.all_objects):
    bpy.data.objects.remove(o, do_unlink=True)
if bpy.context.window:
    bpy.context.window.scene = scene
scene.unit_settings.system = 'METRIC'

def material(name, rgb, metallic=0.0, rough=0.5, emit=None):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = rough
    if emit:
        bsdf.inputs["Emission Color"].default_value = (*emit, 1.0)
        bsdf.inputs["Emission Strength"].default_value = 1.0
    m.diffuse_color = (*rgb, 1.0)
    return m

M_PAINT  = material("Paint",     (0.62, 0.04, 0.03), metallic=0.6, rough=0.25)
M_GLASS  = material("Glass",     (0.05, 0.07, 0.09), rough=0.05)
M_TRIM   = material("Trim",      (0.03, 0.03, 0.035), rough=0.6)
M_CHROME = material("Chrome",    (0.80, 0.80, 0.82), metallic=1.0, rough=0.15)
M_HEAD   = material("Headlight", (0.92, 0.94, 0.98), rough=0.1)
M_TAIL   = material("Taillight", (0.20, 0.02, 0.02), rough=0.1)   # main.c lights it up when braking
M_PLATE  = material("Plate",     (0.90, 0.90, 0.86), rough=0.6)
M_TIRE   = material("Tire",      (0.04, 0.04, 0.045), rough=0.9)
M_RIM    = material("Rim",       (0.70, 0.71, 0.74), metallic=1.0, rough=0.3)
M_WELL   = material("Well",      (0.015, 0.015, 0.015), rough=1.0)

def link(obj):
    scene.collection.objects.link(obj)
    return obj

def apply_modifiers(obj):
    dg = bpy.context.evaluated_depsgraph_get()
    ev = obj.evaluated_get(dg)
    mesh = bpy.data.meshes.new_from_object(ev)
    obj.modifiers.clear()
    old = obj.data
    obj.data = mesh
    bpy.data.meshes.remove(old)

# ---------------------------------------------------------------------------
# Lofted shapes: rings of (y, z) half-profiles placed at stations along X
# ---------------------------------------------------------------------------
def loft(name, stations, half_profile, mats, face_mat=None, cap_mats=(0, 0), subsurf=2):
    """Loft rings along X. face_mat(seg, j) picks the material index of the cage quad between
    station seg and seg+1, profile segment j (0 = bottom centre, counted up the side).
    Materials are set on the coarse cage, so subdivision keeps their borders smooth."""
    bm = bmesh.new()
    rings = []
    for st in stations:
        x = st[0]
        half = half_profile(*st[1:])
        full = half + [(-y, z) for (y, z) in reversed(half[1:-1])]
        rings.append([bm.verts.new((x, y, z)) for (y, z) in full])
    n = len(rings[0])
    nh = len(half_profile(*stations[0][1:])) - 1      # profile segments per side
    for seg, (a, b) in enumerate(zip(rings, rings[1:])):
        for j in range(n):
            k = (j + 1) % n
            f = bm.faces.new((a[j], a[k], b[k], b[j]))
            side_j = j if j < nh else n - 1 - j         # mirror index for the left half
            f.material_index = face_mat(seg, side_j) if face_mat else 0
    bm.faces.new(rings[0]).material_index = cap_mats[0]
    bm.faces.new(list(reversed(rings[-1]))).material_index = cap_mats[1]
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    obj = link(bpy.data.objects.new(name, me))
    for m in mats:
        me.materials.append(m)
    if subsurf:
        obj.modifiers.new("sub", 'SUBSURF').levels = subsurf
        apply_modifiers(obj)
    for p in obj.data.polygons:
        p.use_smooth = True
    return obj

# Lower body: (x, half-width, bottom z, top z)
BODY = [
    ( 2.17, 0.66, 0.36, 0.58),
    ( 2.10, 0.84, 0.26, 0.70),
    ( 1.90, 0.89, 0.21, 0.76),
    ( 1.30, 0.91, 0.19, 0.83),
    ( 0.75, 0.92, 0.19, 0.90),
    ( 0.00, 0.92, 0.19, 0.93),
    (-0.90, 0.92, 0.19, 0.95),
    (-1.70, 0.91, 0.21, 0.98),
    (-2.20, 0.87, 0.27, 0.97),
    (-2.39, 0.78, 0.36, 0.86),
]
def body_profile(hw, zb, zt):
    h = zt - zb
    return [(0, zb), (0.80*hw, zb), (0.97*hw, zb + 0.07), (hw, zb + 0.30*h), (hw, zb + 0.62*h),
            (0.97*hw, zt - 0.07), (0.86*hw, zt), (0.45*hw, zt + 0.01), (0, zt + 0.015)]

PAINT, HEAD, TAIL, TRIM, GLASS = 0, 1, 2, 3, 4
def body_mat(seg, j):
    if seg == 0 and j in (4, 5): return HEAD           # headlights wrap the front corners
    if seg == 8 and j in (4, 5): return TAIL           # taillights wrap the rear corners
    return PAINT

# Greenhouse: (x, half-width at base, base z, roof z)
CABIN = [
    ( 0.92, 0.82, 0.84, 0.89),   # 0 cowl
    ( 0.72, 0.82, 0.84, 1.00),   # 1 windshield foot
    (-0.10, 0.80, 0.86, 1.43),   # 2 windshield top
    (-0.70, 0.80, 0.87, 1.45),   # 3 B-pillar front
    (-0.84, 0.80, 0.87, 1.45),   # 4 B-pillar rear
    (-1.45, 0.80, 0.89, 1.40),   # 5 rear window top
    (-1.95, 0.80, 0.90, 1.02),   # 6 rear window foot
    (-2.06, 0.80, 0.90, 0.94),   # 7 trunk
]
def cabin_profile(hw, zb, zt):
    # 0 centre bottom, 1 base, 2 belt, 3 window top, 4 roof edge, 5 roof centre
    return [(0, zb), (hw, zb), (0.97*hw, zb + 0.10), (0.82*hw, zt - 0.06), (0.66*hw, zt), (0, zt + 0.01)]

def cabin_mat(seg, j):
    if j <= 1: return PAINT                          # under the belt line
    if seg == 0: return TRIM if j == 4 else PAINT     # cowl
    if seg == 1: return {2: GLASS, 3: PAINT, 4: GLASS}[j]   # windshield + A-pillars
    if seg == 3: return TRIM if j == 2 else PAINT     # B-pillar
    if seg in (2, 4): return GLASS if j == 2 else PAINT
    if seg == 5: return GLASS if j == 4 else PAINT    # rear window + C-pillars
    return PAINT

MATS = [M_PAINT, M_HEAD, M_TAIL, M_TRIM, M_GLASS]
body = loft("Body", BODY, body_profile, MATS, body_mat)
cabin = loft("Cabin", CABIN, cabin_profile, MATS, cabin_mat)

# ---------------------------------------------------------------------------
# Wheel arches: a closed pocket per wheel; the cut walls take the dark well material
# ---------------------------------------------------------------------------
ARCH_R = 0.385
for ax in (A, -B):
    for side in (1, -1):
        bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=ARCH_R, depth=1.0,
                                            location=(ax, side*1.05, R + 0.02), rotation=(math.pi/2, 0, 0))
        cutter = bpy.context.active_object
        for c in list(cutter.users_collection): c.objects.unlink(cutter)
        scene.collection.objects.link(cutter)
        cutter.data.materials.append(M_WELL)
        mod = body.modifiers.new("arch", 'BOOLEAN')
        mod.operation = 'DIFFERENCE'
        mod.object = cutter
        mod.solver = 'EXACT'
        mod.material_mode = 'TRANSFER'
        apply_modifiers(body)
        bpy.data.objects.remove(cutter, do_unlink=True)
for p in body.data.polygons:
    p.use_smooth = body.data.materials[p.material_index] != M_WELL
wells = []

# ---------------------------------------------------------------------------
# Details, placed on the body surface by ray casting
# ---------------------------------------------------------------------------
def surface_x(y, z, front=True):
    origin = Vector((4 if front else -4, y, z))
    hit, loc, *_ = body.ray_cast(origin, Vector((-1 if front else 1, 0, 0)))
    return loc.x if hit else (2.1 if front else -2.35)

def surface_y(x, z):
    hit, loc, *_ = body.ray_cast(Vector((x, 3, z)), Vector((0, -1, 0)))
    return loc.y if hit else 0.92

def box(name, loc, size, mat, bevel=0.015, rot=(0, 0, 0)):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc, rotation=rot)
    o = bpy.context.active_object
    o.name = name
    for c in list(o.users_collection): c.objects.unlink(o)
    scene.collection.objects.link(o)
    o.scale = size
    bpy.ops.object.select_all(action='DESELECT')
    o.select_set(True); bpy.context.view_layer.objects.active = o
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel:
        m = o.modifiers.new("bevel", 'BEVEL'); m.width = bevel; m.segments = 3
        apply_modifiers(o)
    o.data.materials.append(mat)
    for p in o.data.polygons: p.use_smooth = True
    return o

details = []
for s in (1, -1):
    # Mirrors on the A-pillar base
    details.append(box("Mirror", (0.52, 1.01*s, 1.00), (0.15, 0.17, 0.11), M_PAINT, bevel=0.03))
    details.append(box("MirrorArm", (0.56, 0.90*s, 0.96), (0.08, 0.10, 0.04), M_TRIM, bevel=0.01))
    # Side sills and door handles
    sy = surface_y(-0.2, 0.30)
    details.append(box("Sill", (-0.15, sy*s, 0.27), (1.75, 0.04, 0.08), M_TRIM, bevel=0.01))
    for hx2 in (0.05, -1.0):
        hy = surface_y(hx2, 0.86)
        details.append(box("Handle", (hx2, hy*s, 0.86), (0.16, 0.025, 0.03), M_CHROME, bevel=0.008))

gx = surface_x(0.0, 0.47)
details.append(box("Grille", (gx + 0.005, 0, 0.47), (0.05, 0.80, 0.15), M_TRIM, bevel=0.02))
details.append(box("FrontPlate", (surface_x(0.0, 0.35) + 0.01, 0, 0.35), (0.02, 0.52, 0.11), M_PLATE, bevel=0.005))
details.append(box("RearPlate", (surface_x(0.0, 0.60, front=False) - 0.01, 0, 0.60), (0.02, 0.52, 0.11), M_PLATE, bevel=0.005))

# ---------------------------------------------------------------------------
# Wheel (one model, rim facing +Y = outward for left wheels; main.c mirrors it for the right)
# ---------------------------------------------------------------------------
def make_wheel():
    parts = []
    bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=R, depth=TIRE_W, rotation=(math.pi/2, 0, 0))
    tire = bpy.context.active_object
    m = tire.modifiers.new("bevel", 'BEVEL'); m.width = 0.045; m.segments = 5; m.limit_method = 'ANGLE'
    apply_modifiers(tire)
    tire.data.materials.append(M_TIRE)
    parts.append(tire)

    face = TIRE_W/2 - 0.012
    bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=0.215, depth=0.012, location=(0, face - 0.03, 0), rotation=(math.pi/2, 0, 0))
    dish = bpy.context.active_object; dish.data.materials.append(M_WELL); parts.append(dish)
    bpy.ops.mesh.primitive_torus_add(major_radius=0.212, minor_radius=0.014, major_segments=48, minor_segments=8,
                                     location=(0, face, 0), rotation=(math.pi/2, 0, 0))
    lip = bpy.context.active_object; lip.data.materials.append(M_RIM); parts.append(lip)
    bpy.ops.mesh.primitive_cylinder_add(vertices=24, radius=0.06, depth=0.04, location=(0, face, 0), rotation=(math.pi/2, 0, 0))
    hub = bpy.context.active_object; hub.data.materials.append(M_RIM); parts.append(hub)
    for k in range(5):
        a = 2*math.pi*k/5
        bpy.ops.mesh.primitive_cube_add(size=1, location=(0.115*math.cos(a), face - 0.005, 0.115*math.sin(a)),
                                        rotation=(0, -a, 0))
        sp = bpy.context.active_object
        sp.scale = (0.17, 0.022, 0.045)
        sp.data.materials.append(M_RIM)
        parts.append(sp)

    for p in parts:
        for c in list(p.users_collection): c.objects.unlink(p)
        scene.collection.objects.link(p)
    bpy.ops.object.select_all(action='DESELECT')
    for p in parts: p.select_set(True)
    bpy.context.view_layer.objects.active = tire
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    bpy.ops.object.join()
    tire.name = "Wheel"
    for p in tire.data.polygons: p.use_smooth = True
    return tire

wheel = make_wheel()

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
os.makedirs(OUT, exist_ok=True)

def export(objs, path, coll_name):
    # Export via a dedicated collection: selection-based export can pick up
    # objects selected in other scenes of the same file.
    coll = bpy.data.collections.get(coll_name) or bpy.data.collections.new(coll_name)
    if coll.name not in scene.collection.children:
        scene.collection.children.link(coll)
    for o in list(coll.objects): coll.objects.unlink(o)
    for o in objs: coll.objects.link(o)
    layer = bpy.context.view_layer.layer_collection.children[coll.name]
    bpy.context.view_layer.active_layer_collection = layer
    bpy.ops.export_scene.gltf(filepath=path, export_format='GLB', use_selection=False,
                              use_active_collection=True, use_active_scene=True,
                              export_yup=True, export_apply=True, export_materials='EXPORT',
                              export_texcoords=False, export_normals=True)
    for o in objs: coll.objects.unlink(o)
    bpy.context.view_layer.active_layer_collection = bpy.context.view_layer.layer_collection

body_parts = [body, cabin] + wells + details
export(body_parts, os.path.join(OUT, "car_body.glb"), "ExportBody")
export([wheel], os.path.join(OUT, "car_wheel.glb"), "ExportWheel")

# Preview placement: four wheels in place (not exported with the body)
for i, (wx, wy) in enumerate(((A, TRACK/2), (A, -TRACK/2), (-B, TRACK/2), (-B, -TRACK/2))):
    w = link(bpy.data.objects.new(f"WheelPreview{i}", wheel.data))
    w.location = (wx, wy, R)
    if wy < 0: w.rotation_euler = (0, 0, math.pi)
wheel.hide_viewport = wheel.hide_render = True

result = {
    "body_tris": sum(len(o.data.polygons) for o in body_parts),
    "wheel_faces": len(wheel.data.polygons),
    "bbox_x": [min((o.matrix_world @ Vector(v)).x for o in [body] for v in o.bound_box),
               max((o.matrix_world @ Vector(v)).x for o in [body] for v in o.bound_box)],
    "out": OUT,
}
print(result)
