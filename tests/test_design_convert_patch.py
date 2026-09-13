# tests/test_design_convert_patch.py
"""Unit tests for design_convert.py::_patch_dae_material_ids against a fake
urdf_usd_converter package in sys.modules -- stdlib only, no Isaac import.
"""
from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import design_convert


class FakeMaterialData:
    def __init__(self) -> None:
        self.name = None
        self.use_material_id = False


class FakeDaeMaterial:
    def __init__(self, id_: str) -> None:
        self.id = id_


class FakeCollada:
    def __init__(self, materials: list[FakeDaeMaterial]) -> None:
        self.materials = materials


class FakeData:
    def __init__(self) -> None:
        self.material_data_list: list[FakeMaterialData] = []


def _fake_store_dae_material_data(mesh_file_path, collada, data) -> None:
    for _ in collada.materials:
        data.material_data_list.append(FakeMaterialData())


class PatchDaeMaterialIdsTest(unittest.TestCase):
    def setUp(self) -> None:
        self._saved_modules = {
            name: sys.modules.get(name)
            for name in ("urdf_usd_converter", "urdf_usd_converter._impl", "urdf_usd_converter._impl.material", "urdf_usd_converter._impl.conversion_collada")
        }
        self.addCleanup(self._restore_modules)

    def _restore_modules(self) -> None:
        for name, module in self._saved_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    def _install_fake_package(self, version: str) -> types.SimpleNamespace:
        package = types.ModuleType("urdf_usd_converter")
        package.__version__ = version
        impl = types.ModuleType("urdf_usd_converter._impl")
        material = types.ModuleType("urdf_usd_converter._impl.material")
        material.store_dae_material_data = _fake_store_dae_material_data
        conversion_collada = types.ModuleType("urdf_usd_converter._impl.conversion_collada")
        conversion_collada.store_dae_material_data = _fake_store_dae_material_data
        sys.modules["urdf_usd_converter"] = package
        sys.modules["urdf_usd_converter._impl"] = impl
        sys.modules["urdf_usd_converter._impl.material"] = material
        sys.modules["urdf_usd_converter._impl.conversion_collada"] = conversion_collada
        return types.SimpleNamespace(package=package, material=material, conversion_collada=conversion_collada)

    def test_guard_skips_when_version_is_not_0_1_3(self) -> None:
        fake = self._install_fake_package("0.2.0")
        design_convert._patch_dae_material_ids()
        self.assertIs(fake.material.store_dae_material_data, _fake_store_dae_material_data)
        self.assertIs(fake.conversion_collada.store_dae_material_data, _fake_store_dae_material_data)

    def test_none_to_id_backfill_and_use_material_id(self) -> None:
        fake = self._install_fake_package("0.1.3")
        design_convert._patch_dae_material_ids()
        data = FakeData()
        collada = FakeCollada([FakeDaeMaterial("mat-a"), FakeDaeMaterial("mat-b")])
        fake.conversion_collada.store_dae_material_data("mesh.dae", collada, data)
        self.assertEqual([item.name for item in data.material_data_list], ["mat-a", "mat-b"])
        self.assertTrue(all(item.use_material_id for item in data.material_data_list))

    def test_1_to_1_zip_pairing_across_multiple_calls(self) -> None:
        fake = self._install_fake_package("0.1.3")
        design_convert._patch_dae_material_ids()
        data = FakeData()
        fake.conversion_collada.store_dae_material_data("mesh1.dae", FakeCollada([FakeDaeMaterial("id1")]), data)
        fake.conversion_collada.store_dae_material_data("mesh2.dae", FakeCollada([FakeDaeMaterial("id2"), FakeDaeMaterial("id3")]), data)
        self.assertEqual([item.name for item in data.material_data_list], ["id1", "id2", "id3"])

    def test_idempotent_second_call_does_not_double_wrap(self) -> None:
        fake = self._install_fake_package("0.1.3")
        design_convert._patch_dae_material_ids()
        patched_once = fake.material.store_dae_material_data
        self.assertTrue(getattr(patched_once, "_design_import_patched", False))
        design_convert._patch_dae_material_ids()
        self.assertIs(fake.material.store_dae_material_data, patched_once)
        self.assertIs(fake.conversion_collada.store_dae_material_data, patched_once)

        data = FakeData()
        collada = FakeCollada([FakeDaeMaterial("only")])
        fake.conversion_collada.store_dae_material_data("mesh.dae", collada, data)
        self.assertEqual([item.name for item in data.material_data_list], ["only"])


if __name__ == "__main__":
    unittest.main()
