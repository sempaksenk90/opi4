#!/bin/bash
# ============================================================
# install_dtsi_patch.sh
# Installs sun60iw2p1 DTSI patch into orangepi-build userpatches
# Run this from the root of your orangepi-build directory:
#   cd orangepi-build
#   bash install_dtsi_patch.sh
# ============================================================

set -e

PATCH_NAME="0001-dts-sun60iw2p1-enable-xhci2-usb3-peripheral.patch"
PATCH_DIR="userpatches/kernel/sun60iw2-current"

# ── 1. Verify we are inside orangepi-build ────────────────
if [[ ! -f "build.sh" || ! -d "external" ]]; then
    echo "[ERROR] Run this script from the root of your orangepi-build directory."
    exit 1
fi

# ── 2. Create the userpatches directory ───────────────────
echo "[INFO] Creating patch directory: ${PATCH_DIR}"
mkdir -p "${PATCH_DIR}"

# ── 3. Write the patch file ───────────────────────────────
echo "[INFO] Writing patch: ${PATCH_DIR}/${PATCH_NAME}"
cat > "${PATCH_DIR}/${PATCH_NAME}" << 'PATCH'
From: Custom Patch <user@localhost>
Date: Sun, 24 May 2026 00:00:00 +0000
Subject: [PATCH] dts: sun60iw2p1: enable xhci2 USB3 peripheral mode and fix PHY

Enable xhci2 controller in peripheral mode with corrected clock rates,
enable u2phy, disable combo0 DP PHY and combo1 PCIe PHY to route
the SerDes lanes to USB3 instead.

Changes:
- xhci2: dr_mode otg -> peripheral
- xhci2: add CLK_USB2_MF to assigned-clocks at 400MHz
- xhci2: maximum-speed super-speed-plus -> super-speed
- xhci2: status disabled -> okay
- u2phy: status disabled -> okay
- combo0_dp: status okay -> disabled
- combo1_pcie: status okay -> disabled
---
 .../boot/dts/allwinner/sun60iw2p1.dtsi        | 22 +++++++++----------
 1 file changed, 11 insertions(+), 11 deletions(-)

diff --git a/arch/arm64/boot/dts/allwinner/sun60iw2p1.dtsi b/arch/arm64/boot/dts/allwinner/sun60iw2p1.dtsi
--- a/arch/arm64/boot/dts/allwinner/sun60iw2p1.dtsi
+++ b/arch/arm64/boot/dts/allwinner/sun60iw2p1.dtsi
@@ -4094,22 +4094,21 @@ xhci2: xhci2-controller@6a00000 {
 			compatible = "snps,dwc3";
 			reg = <0x0 0x06a00000 0x0 0x100000>;
 			interrupts = <GIC_SPI 155 IRQ_TYPE_LEVEL_HIGH>;
-			dr_mode = "otg";
+			dr_mode = "peripheral";
 			clocks = <&ccu CLK_USB2_MF>,
 				 <&ccu CLK_USB2_U2_REF>,
 				 <&ccu CLK_USB2_SUSPEND>;
 			clock-names = "bus_clk", "ref_clk", "suspend";
-			assigned-clocks = <&ccu CLK_USB2_SUSPEND>;
-			assigned-clock-rates = <24000000>;
+			assigned-clocks = <&ccu CLK_USB2_SUSPEND>, <&ccu CLK_USB2_MF>;
+			assigned-clock-rates = <24000000>, <400000000>;
 			resets = <&ccu RST_USB_2>;
 			reset-names = "hci";
 			power-domains = <&pd SUN60IW2_PCK_USB2>;
-			maximum-speed = "super-speed-plus";
+			maximum-speed = "super-speed";
 			phy_type = "utmi";
 			snps,dis_enblslpm_quirk;
 			snps,dis-u1-entry-quirk;
 			snps,dis-u2-entry-quirk;
 			snps,dis_u3_susphy_quirk;
 			snps,dis_u2_susphy_quirk;
 			phys = <&u2phy>, <&combo0_usb>;
 			phy-names = "usb2-phy", "usb3-phy";
-			status = "disabled";
+			status = "okay";
 		};
 	};
@@ -4123,7 +4123,7 @@ clock-names = "res_dcap";
 		aw,rext_mode = <2>;
 		aw,phy_tune_param = <0x143338D6>;
 		#phy-cells = <0>;
-		status = "disabled";
+		status = "okay";
 	};
@@ -4153,7 +4153,7 @@ reg = <0x0 0x06c01000 0x0 0xa00>,
 	combo0_dp: combo0-dp-phy {
 		#phy-cells = <0>;
-		status = "okay";
+		status = "disabled";
 	};
@@ -4167,7 +4167,7 @@ combo1_usb: combo1-usb-phy {
 	combo1_pcie: combo1-pcie-phy {
 		#phy-cells = <0>;
-		status = "okay";
+		status = "disabled";
 	};
PATCH

echo ""
echo "[OK] Patch installed at: ${PATCH_DIR}/${PATCH_NAME}"
echo ""
echo "── Next steps ─────────────────────────────────────────"
echo "  Build kernel only (fast):"
echo "    sudo ./build.sh BOARD=orangepi4pro BRANCH=current BUILD_ONLY=kernel"
echo ""
echo "  Full image build:"
echo "    sudo ./build.sh BOARD=orangepi4pro BRANCH=current"
echo ""
echo "  Watch build log for confirmation line:"
echo "    [ o.k. ] * [u] ${PATCH_NAME}"
echo "────────────────────────────────────────────────────────"
