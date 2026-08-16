.. _nrf54l15_auracaster:

nRF54L15: Auracaster (LE Audio broadcast source)
################################################

.. contents::
   :local:
   :depth: 2

Overview
********

This sample turns an nRF54L15 into an Auracast\ :sup:`TM` broadcast source.
It captures audio from an on-board PDM microphone, encodes it with the LC3
codec running on the application core, and transmits it as a Broadcast
Isochronous Group (BIG).

The sample starts extended advertising carrying the Broadcast Audio
Announcement and the Broadcast Name, periodic advertising carrying the
Broadcast Audio Source Endpoint (BASE), and then the BIG itself.

It derives from the Zephyr :zephyr:code-sample:`bluetooth_bap_broadcast_source`
sample, with the mock sine/tone generator replaced by a real PDM capture path
and the Zephyr LIBLC3 encoder replaced by the nrfxlib T2 software LC3 encoder
(``CONFIG_SW_CODEC_LC3_T2_SOFTWARE``).

Audio configuration
===================

* LC3 preset ``BT_BAP_LC3_BROADCAST_PRESET_16_2_1`` (16 kHz, 10 ms frames)
* Two BIS streams (front left / front right) in a single subgroup
* Mono PDM capture, duplicated across both streams

Requirements
************

One of the following boards:

.. list-table::
   :header-rows: 1

   * - Board target
     - Microphone
   * - ``nrf54l15dk/nrf54l15/cpuapp``
     - External PDM microphone on P1.12 (CLK) and P1.13 (DIN)
   * - ``xiao_nrf54l15/nrf54l15/cpuapp``
     - On-board PDM microphone (Seeed XIAO nRF54L15 Sense)

To listen to the stream you also need an Auracast receiver. Prebuilt nRF5340
Audio DK BIS headset images are included in this directory as
:file:`nrf5340_bis_headset_app.hex` and :file:`nrf5340_bis_headset_net.hex`.

Building and running
********************

Seeed XIAO nRF54L15 Sense
=========================

The board is in the Zephyr tree as of NCS v3.4.0, so no ``BOARD_ROOT`` is
needed:

.. code-block:: console

   west build -b xiao_nrf54l15/nrf54l15/cpuapp samples/nrf54l15_auracaster

The on-board microphone needs no overlay work: the board files already assign
the PDM pins and apply the ``dmic_dev`` label, and the microphone power rail
(``pdm_imu_pwr``, P0.01) is brought up automatically before the DMIC driver
initializes.

This build advertises the Broadcast Name ``XIAO_54L15_Sense``.

nRF54L15 DK
===========

.. code-block:: console

   west build -b nrf54l15dk/nrf54l15/cpuapp samples/nrf54l15_auracaster

Connect a PDM microphone to P1.12 (PDM_CLK) and P1.13 (PDM_DIN). This build
advertises the Broadcast Name ``nRF54L15_PDM``.

Configuration
*************

``CONFIG_BROADCAST_CODE``
   Non-empty string encrypts the broadcast with that code (1-16 octets).

``CONFIG_STATIC_BROADCAST_ID`` / ``CONFIG_BROADCAST_ID``
   Use a fixed 3-octet broadcast ID instead of a random one. Defaults to
   ``0x123456``.

``CONFIG_BT_DEVICE_NAME``
   Also used as the advertised Broadcast Name.

Limitations
***********

* The application uses ``DT_NODELABEL(dmic_dev)`` unconditionally, so it only
  builds for board targets whose devicetree provides a PDM node under that
  label. The nRF5340 and nRF52 overlays inherited from the upstream Zephyr
  sample do not, and those targets do not build.
* Capture is mono. Both BIS streams carry the same audio, tagged as front left
  and front right.
