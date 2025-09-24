"""Behavioral tests for the network access point provisioning service.

We assert that the service prepares hostapd, dnsmasq, and Avahi so that
robots form an isolated DDS-ready WLAN.
"""

from pathlib import Path


def _load_network_ap_script() -> str:
    """Return the network_ap provisioning script as text."""

    repo_root = Path(__file__).resolve().parents[2]
    script_path = repo_root / "provision" / "services" / "network_ap.sh"
    return script_path.read_text(encoding="utf-8")


def test_network_ap_queues_wireless_stack() -> None:
    """Provisioning should queue hostapd, dnsmasq, and Avahi."""

    script_text = _load_network_ap_script()

    assert "hostapd" in script_text
    assert "dnsmasq" in script_text
    assert "avahi-daemon" in script_text


def test_network_ap_writes_default_configuration_file() -> None:
    """Provisioning should stamp a defaults file with SSID and addressing knobs."""

    script_text = _load_network_ap_script()

    assert "/etc/default/psyched-network-ap" in script_text
    assert "PSY_AP_SSID" in script_text
    assert "PSY_AP_ADDRESS" in script_text


def test_network_ap_launcher_defines_hostapd_and_dnsmasq_configs() -> None:
    """Launcher should render hostapd and dnsmasq configuration snippets."""

    script_text = _load_network_ap_script()

    assert "network-ap.hostapd.conf" in script_text
    assert "network-ap.dnsmasq.conf" in script_text
    assert "wpa_passphrase=${AP_PASSWORD}" in script_text
    assert "dhcp-range=${AP_DHCP_RANGE}" in script_text


def test_network_ap_installs_avahi_service_manifest() -> None:
    """Provisioning should register an Avahi service for DDS peers."""

    script_text = _load_network_ap_script()

    assert "psyched-dds.service" in script_text
    assert "_dds._udp" in script_text
