from setuptools import setup

PKG = "kinova_dual_arm_dmp_insertion"

setup(
    name=PKG,
    version="0.1.0",
    packages=[PKG],
    # the routes and the human prior travel inside the python package
    package_data={PKG: ["prior_torsional_C.npz", "routes/*/*.json", "routes/*/*_trace.npz"]},
    data_files=[("share/ament_index/resource_index/packages", [f"resource/{PKG}"]),
                (f"share/{PKG}", ["package.xml", "README.md"]),
                (f"share/{PKG}/config", ["config/cell.yaml"])],
    install_requires=["setuptools"],
    zip_safe=False,
    entry_points={"console_scripts": [f"list = {PKG}.main:list_entry",
                                      f"check = {PKG}.main:check_entry",
                                      f"insert = {PKG}.main:insert_entry"]},
)
