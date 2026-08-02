import xml.etree.ElementTree as ET

WORLD_NAME = "arm_world"
SDF_VERSION = "1.10"

TABLE_TOP = 0.75
TABLE_SIZE = (1.0, 0.7, 0.03)
LEG_SIZE = (0.06, 0.06, 0.72)
BLOCK_SIZE = 0.02
BLOCK_Z = TABLE_TOP + BLOCK_SIZE / 2.0

# Blocks spread out near the arm base so the arm can reach them from above
# without self-collision, and spaced enough for the jaws to envelop freely.
BLOCKS = [
    ("block_red",   (0.14, 0.0),  0.0, (1.0, 0.15, 0.15)),
    ("block_green", (0.14, 0.06), 0.0, (0.15, 1.0, 0.15)),
    ("block_blue",  (0.14, -0.06), 0.0, (0.15, 0.3, 1.0)),
]

BUCKET_FOOTPRINT = 0.09
BUCKET_WALL = 0.01
BUCKET_HEIGHT = 0.08

# Buckets spaced 20 cm+ apart and within the ~0.31 m tool-down radius so
# the IK closing chain can sample real goal states for every colour.
BUCKETS = [
    ("bucket_red",   (0.22, 0.15),  (1.0, 0.2, 0.2, 1)),
    ("bucket_green", (0.24, 0.0),   (0.2, 1.0, 0.2, 1)),
    ("bucket_blue",  (0.22, -0.15), (0.2, 0.35, 1.0, 1)),
]


def el(tag, **attrs):
    e = ET.Element(tag)
    for k, v in attrs.items():
        e.set(k, str(v))
    return e


def box_geom(size):
    g = el("geometry")
    b = el("box")
    s = el("size")
    s.text = size
    b.append(s)
    g.append(b)
    return g


def make_box_collision(size, mu=0.6, name="collision"):
    c = el("collision", name=name)
    c.append(box_geom(size))
    surface = el("surface")
    friction = el("friction")
    fmu = el("mu")
    fmu.text = str(mu)
    friction.append(fmu)
    surface.append(friction)
    c.append(surface)
    return c


def make_box_visual(size, rgba, name="visual"):
    v = el("visual", name=name)
    v.append(box_geom(size))
    m = el("material")
    text = rgba if isinstance(rgba, str) else " ".join(str(x) for x in rgba)
    a = el("ambient")
    a.text = text
    d = el("diffuse")
    d.text = text
    m.append(a)
    m.append(d)
    v.append(m)
    return v


def make_inertial_box(size, mass):
    i = el("inertial")
    m = el("mass")
    m.text = str(mass)
    i.append(m)
    inertia = el("inertia")
    lx, ly, lz = size
    ixx = mass / 12.0 * (ly * ly + lz * lz)
    iyy = mass / 12.0 * (lx * lx + lz * lz)
    izz = mass / 12.0 * (lx * lx + ly * ly)
    for tag, val in (("ixx", ixx), ("ixy", 0.0), ("ixz", 0.0),
                     ("iyy", iyy), ("iyz", 0.0), ("izz", izz)):
        e = el(tag)
        e.text = f"{val:.6e}"
        inertia.append(e)
    i.append(inertia)
    return i


def floor_model():
    m = el("model", name="floor")
    m.append(el("static"))
    m[0].text = "true"
    p = el("pose")
    p.text = "0 0 -0.05 0 0 0"
    m.append(p)
    l = el("link", name="link")
    l.append(make_box_collision("10 10 0.1"))
    l.append(make_box_visual("10 10 0.1", "0.55 0.58 0.62 1"))
    m.append(l)
    return m


def table_model():
    m = el("model", name="table")
    m.append(el("static"))
    m[0].text = "true"
    p = el("pose")
    p.text = "0 0 0 0 0 0"
    m.append(p)
    l = el("link", name="top")
    tp = el("pose")
    tp.text = f"0 0 {TABLE_TOP - TABLE_SIZE[2] / 2.0} 0 0 0"
    l.append(tp)
    l.append(make_box_collision(
        f"{TABLE_SIZE[0]} {TABLE_SIZE[1]} {TABLE_SIZE[2]}"))
    l.append(make_box_visual(
        f"{TABLE_SIZE[0]} {TABLE_SIZE[1]} {TABLE_SIZE[2]}",
        "0.55 0.38 0.2 1"))
    m.append(l)
    for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (-1, 1), (1, 1)]):
        leg = el("link", name=f"leg_{i}")
        lp = el("pose")
        lp.text = (f"{sx * (TABLE_SIZE[0] / 2.0 - 0.03)} "
                   f"{sy * (TABLE_SIZE[1] / 2.0 - 0.03)} "
                   f"{LEG_SIZE[2] / 2.0} 0 0 0")
        leg.append(lp)
        leg.append(make_box_collision(
            f"{LEG_SIZE[0]} {LEG_SIZE[1]} {LEG_SIZE[2]}"))
        leg.append(make_box_visual(
            f"{LEG_SIZE[0]} {LEG_SIZE[1]} {LEG_SIZE[2]}",
            "0.45 0.3 0.16 1"))
        m.append(leg)
    return m


def block_model(name, xy, yaw, rgba):
    m = el("model", name=name)
    p = el("pose")
    p.text = f"{xy[0]} {xy[1]} {BLOCK_Z} 0 0 {yaw}"
    m.append(p)
    l = el("link", name="link")
    l.append(make_inertial_box((BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE), 0.05))
    l.append(make_box_collision(
        f"{BLOCK_SIZE} {BLOCK_SIZE} {BLOCK_SIZE}", mu=0.6))
    l.append(make_box_visual(
        f"{BLOCK_SIZE} {BLOCK_SIZE} {BLOCK_SIZE}", rgba))
    m.append(l)
    return m


def bucket_model(name, xy, rgba):
    m = el("model", name=name)
    m.append(el("static"))
    m[0].text = "true"
    p = el("pose")
    p.text = f"{xy[0]} {xy[1]} {TABLE_TOP} 0 0 0"
    m.append(p)
    half = BUCKET_FOOTPRINT / 2.0
    walls = (
        ("wall_n", (0.0, half - BUCKET_WALL / 2.0),
         f"{BUCKET_FOOTPRINT} {BUCKET_WALL} {BUCKET_HEIGHT}"),
        ("wall_s", (0.0, -(half - BUCKET_WALL / 2.0)),
         f"{BUCKET_FOOTPRINT} {BUCKET_WALL} {BUCKET_HEIGHT}"),
        ("wall_e", (half - BUCKET_WALL / 2.0, 0.0),
         f"{BUCKET_WALL} {BUCKET_FOOTPRINT} {BUCKET_HEIGHT}"),
        ("wall_w", (-(half - BUCKET_WALL / 2.0), 0.0),
         f"{BUCKET_WALL} {BUCKET_FOOTPRINT} {BUCKET_HEIGHT}"),
    )
    for wname, (wx, wy), wsize in walls:
        l = el("link", name=wname)
        lp = el("pose")
        lp.text = f"{wx} {wy} {BUCKET_HEIGHT / 2.0} 0 0 0"
        l.append(lp)
        l.append(make_box_collision(wsize))
        l.append(make_box_visual(wsize, rgba))
        m.append(l)
    return m


def gen_sdf():
    root = el("sdf", version=SDF_VERSION)
    world = el("world", name=WORLD_NAME)
    physics = el("physics", type="dart")
    ms = el("max_step_size")
    ms.text = "0.001"
    rtf = el("real_time_factor")
    rtf.text = "1.0"
    physics.append(ms)
    physics.append(rtf)
    world.append(physics)
    scene = el("scene")
    ambient = el("ambient")
    ambient.text = "0.45 0.45 0.45 1"
    bg = el("background")
    bg.text = "0.72 0.75 0.8 1"
    shadows = el("shadows")
    shadows.text = "true"
    scene.append(ambient)
    scene.append(bg)
    scene.append(shadows)
    world.append(scene)
    sun = el("light", type="directional", name="sun")
    lp = el("pose")
    lp.text = "0 0 3 0 0 0"
    sun.append(lp)
    diffuse = el("diffuse")
    diffuse.text = "1 1 1 1"
    specular = el("specular")
    specular.text = "0.5 0.5 0.5 1"
    direction = el("direction")
    direction.text = "-0.5 -0.7 -1"
    sun.append(diffuse)
    sun.append(specular)
    sun.append(direction)
    world.append(sun)
    table_light = el("light", type="point", name="table_light")
    tlp = el("pose")
    tlp.text = "0 0 2.2 0 0 0"
    table_light.append(tlp)
    td = el("diffuse")
    td.text = "1 1 1 1"
    table_light.append(td)
    world.append(table_light)
    world.append(floor_model())
    world.append(table_model())
    for name, xy, yaw, rgba in BLOCKS:
        world.append(block_model(name, xy, yaw, rgba))
    for name, xy, rgba in BUCKETS:
        world.append(bucket_model(name, xy, rgba))
    gui = el("gui")

    def prop(plugin, key, value, ptype="string"):
        p = el("property", type=ptype, key=key)
        p.text = str(value)
        plugin.append(p)

    def hidden(name, filename):
        p = el("plugin", filename=filename, name=name)
        gz = el("gz-gui")
        prop(gz, "resizable", "false", "bool")
        prop(gz, "width", "5", "double")
        prop(gz, "height", "5", "double")
        prop(gz, "state", "floating")
        prop(gz, "showTitleBar", "false", "bool")
        p.append(gz)
        return p

    scene = el("plugin", filename="MinimalScene", name="3D View")
    gz = el("gz-gui")
    prop(gz, "showTitleBar", "false", "bool")
    prop(gz, "state", "docked")
    scene.append(gz)
    engine = el("engine")
    engine.text = "ogre2"
    scene.append(engine)
    sscene = el("scene")
    sscene.text = "scene"
    scene.append(sscene)
    ambient = el("ambient_light")
    ambient.text = "0.4 0.4 0.4"
    scene.append(ambient)
    bg = el("background_color")
    bg.text = "0.8 0.8 0.8"
    scene.append(bg)
    cam = el("camera_pose")
    cam.text = "-1.2 0.4 1.4 0 0.5 0"
    scene.append(cam)
    gui.append(scene)

    gui.append(hidden("Scene Manager", "GzSceneManager"))
    gui.append(hidden("Interactive view control", "InteractiveViewControl"))
    gui.append(hidden("Entity context menu", "EntityContextMenuPlugin"))
    gui.append(hidden("Select Entities", "SelectEntities"))

    feed = el("plugin", filename="ImageDisplay", name="Camera feed")
    gz = el("gz-gui")
    prop(gz, "title", "Camera feed")
    prop(gz, "state", "floating")
    prop(gz, "x", "80", "double")
    prop(gz, "y", "60", "double")
    prop(gz, "width", "640", "double")
    prop(gz, "height", "480", "double")
    feed.append(gz)
    topic = el("topic")
    topic.text = "/camera_table/detected"
    feed.append(topic)
    gui.append(feed)

    world.append(gui)
    root.append(world)
    return root


if __name__ == "__main__":
    ET.indent(gen_sdf())
    print(ET.tostring(gen_sdf(), encoding="unicode"))
