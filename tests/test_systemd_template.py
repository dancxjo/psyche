import pathlib


def test_psyched_service_template_sets_runtime_dir():
    template = pathlib.Path("provision/systemd/install_units.sh").read_text()
    assert "Environment=XDG_RUNTIME_DIR=/run/user/%U" in template
    assert "ExecStartPre=/bin/mkdir -p /run/user/%U" in template
    assert "ExecStartPre=/bin/chmod 700 /run/user/%U" in template

