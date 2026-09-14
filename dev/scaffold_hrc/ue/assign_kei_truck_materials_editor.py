# Unreal Editor Python (UE 5.3). Assign solid M_Kei_* to SM_SuzukiCarry1993.
#
# Crash note (UE 5.3.2 + UnrealCV): rewriting StaticMesh.static_materials,
# saving the .uasset during PIE, then UnrealCV-destroying BP_KeiTruck crashed
# the editor. Never destroy ScaffoldHrc_truck. Never save the mesh in PIE.
# In PIE: in-memory sm.set_material + live component overrides only.

from __future__ import annotations

import unreal

DEST = "/Game/SimWorld/Props/KeiTruck"
MESH_PATH = f"{DEST}/SM_SuzukiCarry1993"
BP_PATH = f"{DEST}/BP_KeiTruck"
TRUCK_LABEL = "ScaffoldHrc_truck"
DEBUG_PATH = r"C:\UEProjects\SimWorld\Saved\kei_mat_debug.txt"

MATS = {
    "paint": f"{DEST}/M_Kei_Paint",
    "glass": f"{DEST}/M_Kei_Glass",
    "tyre": f"{DEST}/M_Kei_Tyre",
    "chrome": f"{DEST}/M_Kei_Chrome",
    "plate": f"{DEST}/M_Kei_Plate",
}

GLASS = {"glass", "window", "windshield"}
TYRE = {"tyre", "tire", "rubber", "black"}
CHROME = {"chrome", "headlight", "taillight", "blink", "light"}
PLATE = {"vermont", "plate", "licence", "license"}

_LOG: list[str] = []


def _log(msg: str) -> None:
    line = f"[KeiTruckMat] {msg}"
    _LOG.append(line)
    print(line)
    unreal.log(line)


def _flush() -> None:
    try:
        with open(DEBUG_PATH, "w", encoding="utf-8") as handle:
            handle.write("\n".join(_LOG) + "\n")
    except Exception as exc:
        unreal.log_error(f"[KeiTruckMat] debug write failed: {exc}")


def _load(path: str):
    candidates = (path, f"{path}.{path.rsplit('/', 1)[-1]}")
    for candidate in candidates:
        try:
            asset = unreal.load_asset(candidate)
        except Exception:
            asset = None
        if asset is not None:
            return asset
        try:
            asset = unreal.EditorAssetLibrary.load_asset(candidate)
        except Exception:
            asset = None
        if asset is not None:
            return asset
    return None


def _log_kei_folder() -> None:
    try:
        names = list(unreal.EditorAssetLibrary.list_assets(DEST, recursive=False))
        _log(f"list_assets {DEST} count={len(names)}")
        for name in names[:40]:
            _log(f"  asset {name}")
    except Exception as exc:
        _log(f"list_assets failed: {exc}")
    try:
        registry = unreal.AssetRegistryHelpers.get_asset_registry()
        datas = list(registry.get_assets_by_path(DEST, recursive=True))
        _log(f"asset_registry {DEST} count={len(datas)}")
        for data in datas[:40]:
            _log(f"  reg {data.package_name} {data.asset_name} {data.asset_class_path}")
    except Exception as exc:
        _log(f"asset_registry failed: {exc}")


def _slot_kind(name: str) -> str:
    n = (name or "").lower().replace(".", "_").replace(" ", "_")
    for token, kind in (
        (GLASS, "glass"),
        (TYRE, "tyre"),
        (CHROME, "chrome"),
        (PLATE, "plate"),
    ):
        if any(t in n for t in token):
            return kind
    return "paint"


def _in_pie() -> bool:
    try:
        sub = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
        world = sub.get_game_world()
        return world is not None
    except Exception:
        return False


def _worlds():
    worlds = []
    try:
        sub = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
        getters = ("get_game_world",) if _in_pie() else ("get_game_world", "get_editor_world")
        for getter in getters:
            fn = getattr(sub, getter, None)
            if fn is None:
                continue
            world = fn()
            if world is not None:
                worlds.append(world)
    except Exception as exc:
        _log(f"world lookup failed: {exc}")
    return worlds


def _mesh_path(mesh) -> str:
    if mesh is None:
        return ""
    try:
        return str(mesh.get_path_name())
    except Exception:
        return ""


def _comp_mesh(comp):
    for attr in ("static_mesh",):
        try:
            mesh = getattr(comp, attr, None)
            if mesh is not None:
                return mesh
        except Exception:
            pass
    try:
        return comp.get_editor_property("static_mesh")
    except Exception:
        return None


def _is_truck_actor(actor) -> bool:
    try:
        label = str(actor.get_actor_label())
        name = str(actor.get_name())
    except Exception:
        return False
    if TRUCK_LABEL in label or TRUCK_LABEL in name:
        return True
    if "KeiTruck" in label or "KeiTruck" in name:
        return True
    if "SuzukiCarry" in label or "SuzukiCarry" in name:
        return True
    try:
        comps = actor.get_components_by_class(unreal.StaticMeshComponent)
    except Exception:
        return False
    for comp in comps:
        if "SuzukiCarry" in _mesh_path(_comp_mesh(comp)):
            return True
    return False


def _load_bp_class():
    paths = (
        f"{BP_PATH}.{BP_PATH.rsplit('/', 1)[-1]}_C",
        f"{BP_PATH}_C",
        BP_PATH,
    )
    try:
        cls = unreal.EditorAssetLibrary.load_blueprint_class(BP_PATH)
        if cls is not None:
            return cls
    except Exception as exc:
        _log(f"load_blueprint_class failed: {exc}")
    for path in paths:
        for loader_name in ("load_class", "load_object"):
            loader = getattr(unreal, loader_name, None)
            if loader is None:
                continue
            try:
                cls = loader(None, path)
            except Exception:
                cls = None
            if cls is not None:
                _log(f"bp class via {loader_name} {path}")
                return cls
    return None


def _scan_named_actors():
    hits = []
    for world in _worlds():
        try:
            all_actors = list(unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor))
        except Exception as exc:
            _log(f"scan actors failed: {exc}")
            continue
        _log(f"scan {world.get_name()} n={len(all_actors)}")
        for actor in all_actors:
            try:
                name = str(actor.get_name())
                label = str(actor.get_actor_label())
            except Exception:
                continue
            blob = name + " " + label
            if TRUCK_LABEL in blob or "KeiTruck" in blob or "SuzukiCarry" in blob:
                hits.append(actor)
    _log(f"name hits={len(hits)}")
    return hits


def _iter_truck_actors():
    seen = set()
    actors = []
    bp_cls = _load_bp_class()
    _log(f"bp_class={bp_cls}")
    if bp_cls is not None:
        for world in _worlds():
            try:
                found = list(unreal.GameplayStatics.get_all_actors_of_class(world, bp_cls))
                _log(f"bp actors in {world.get_name()}={len(found)}")
                actors.extend(found)
            except Exception as exc:
                _log(f"get_all_actors_of_class failed: {exc}")
    if not actors:
        actors.extend(_scan_named_actors())
    if not actors:
        getter = getattr(unreal.EditorLevelLibrary, "get_all_level_actors", None)
        if getter is not None:
            try:
                actors.extend(list(getter()))
                _log(f"fallback level_actors={len(actors)}")
            except Exception as exc:
                _log(f"get_all_level_actors failed: {exc}")
    for actor in actors:
        try:
            key = actor.get_path_name()
        except Exception:
            continue
        if key in seen:
            continue
        seen.add(key)
        if _is_truck_actor(actor):
            yield actor


def _collect_slots(sm):
    rows = []
    try:
        slots = list(sm.static_materials)
    except Exception as exc:
        _log(f"static_materials read failed: {exc}")
        slots = []
    _log(f"static_materials len={len(slots)}")
    for i, slot in enumerate(slots):
        name = f"slot_{i}"
        try:
            name = str(slot.material_slot_name)
        except Exception:
            pass
        current = None
        try:
            current = slot.material_interface
        except Exception:
            pass
        rows.append((i, name, current))
    if rows:
        return rows
    for i in range(64):
        try:
            current = sm.get_material(i)
        except Exception:
            break
        if current is None and i > 0 and not rows:
            break
        if current is None and i > 8:
            break
        rows.append((i, f"slot_{i}", current))
    _log(f"probed mesh materials={len(rows)}")
    return rows


def _apply_to_component(comp, chosen: list) -> int:
    n = 0
    n_comp = 0
    getter = getattr(comp, "get_num_materials", None)
    if callable(getter):
        try:
            n_comp = int(getter())
        except Exception:
            n_comp = 0
    count = max(n_comp, len(chosen), 1)
    _log(f"  component materials={n_comp} apply={count} mesh={_mesh_path(_comp_mesh(comp))}")
    for i in range(count):
        mat = chosen[i] if i < len(chosen) else chosen[0]
        if mat is None:
            continue
        try:
            comp.set_material(i, mat)
            n += 1
        except Exception as exc:
            _log(f"  component set_material({i}) failed: {exc}")
    try:
        comp.mark_render_state_dirty()
    except Exception:
        pass
    return n


def main() -> None:
    try:
        _log_kei_folder()
        sm = _load(MESH_PATH)
        if sm is None:
            raise RuntimeError(f"missing {MESH_PATH}")

        loaded = {k: _load(p) for k, p in MATS.items()}
        missing = [k for k, m in loaded.items() if m is None]
        _log(f"loaded mats missing={missing}")
        if missing:
            raise RuntimeError(f"missing materials {missing}")

        pie = _in_pie()
        rows = _collect_slots(sm)
        _log(f"mesh slots={len(rows)} pie={pie}")
        chosen = []
        for i, name, current in rows:
            kind = _slot_kind(name)
            mat = loaded[kind]
            chosen.append(mat)
            cur_name = ""
            try:
                if current is not None:
                    cur_name = str(current.get_name())
            except Exception:
                pass
            _log(f"  [{i}] {name!r} -> {kind} was={cur_name}")

        if not chosen:
            chosen = [loaded["paint"]]
            _log("no slots; using paint for index 0")

        for i, mat in enumerate(chosen):
            try:
                sm.set_material(i, mat)
            except Exception as exc:
                _log(f"sm.set_material({i}) failed: {exc}")
        if not pie:
            try:
                unreal.EditorAssetLibrary.save_asset(MESH_PATH, only_if_is_dirty=True)
                _log("saved mesh asset")
            except Exception as exc:
                _log(f"save skipped: {exc}")
        else:
            _log("PIE: in-memory sm.set_material only (no save, no static_materials rewrite)")

        n_found = 0
        n_set = 0
        for actor in _iter_truck_actors():
            n_found += 1
            label = actor.get_actor_label()
            path = actor.get_path_name()
            _log(f"actor {label} {path}")
            comps = actor.get_components_by_class(unreal.StaticMeshComponent)
            _log(f"  smc_count={len(comps)}")
            for comp in comps:
                n_set += _apply_to_component(comp, chosen)
        _log(f"done slots={len(chosen)} actors={n_found} set={n_set}")
    except Exception as exc:
        _log(f"FAIL: {exc}")
        raise
    finally:
        _flush()


if __name__ == "__main__":
    main()
