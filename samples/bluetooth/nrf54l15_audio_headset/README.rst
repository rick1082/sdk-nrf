.. _nrf54l15_audio_headset:

Bluetooth: nRF54L15 Audio Headset
#################################

.. contents::
   :local:
   :depth: 2

The nRF54L15 Audio Headset sample demonstrates an LE Audio headset that a phone can configure for either unicast audio using Connected Isochronous Streams (CIS) or broadcast audio using Broadcast Isochronous Streams (BIS).
The received audio is decoded and played on a TLV320 DAC connected over I2S.

Requirements
************

The sample supports the following development kits:

.. table-from-sample-yaml::

The sample requires a TLV320 DAC connected to the development kit as follows:

.. list-table::
   :header-rows: 1

   * - Signal
     - nRF54L15 DK pin
   * - I2S MCK (to the DAC)
     - P1.07
   * - I2S SCK (from the DAC)
     - P1.06
   * - I2S LRCK (from the DAC)
     - P1.05
   * - I2S SDOUT (to the DAC)
     - P1.04
   * - I2C SDA
     - P1.11
   * - I2C SCL
     - P1.12
   * - DAC reset
     - P1.14 (**LED3**)

The I2S pins are shared with UART20 on the development kit, so UART20 is disabled.
The logs are output over Segger RTT.

To test the sample, you need a phone or another device acting as Common Audio Profile (CAP) Initiator and Broadcast Assistant, for example an Android phone with LE Audio and Audio Sharing support.
The sample also requires a connection to a computer with an RTT viewer, for example the J-Link RTT Viewer or the RTT terminal of the nRF Connect for VS Code extension.

Overview
********

The sample implements the Telephony and Media Audio Profile (TMAP) Call Terminal (CT), Unicast Media Receiver (UMR), and Broadcast Media Receiver (BMR) roles.
It advertises connectable and waits for a CAP Initiator or Broadcast Assistant to connect.

Android only offers its Broadcast Assistant (Audio Sharing) for devices that support both unicast and broadcast audio.
The connected phone can configure either of the following:

* CIS - The sample uses the Basic Audio Profile (BAP) Unicast Server role with one sink and one source Audio Stream Endpoint (ASE).
  The audio received on the sink ASE is played on both outputs, and the source ASE sends dummy (all zero) SDUs.
* BIS - The sample uses the BAP Scan Delegator and Broadcast Sink roles.
  The Broadcast Assistant adds a broadcast source, optionally transfers the periodic advertising sync (PAST), and provides the Broadcast Code for encrypted broadcasts.
  Up to two BISes of one subgroup are played on the left and right outputs, using their channel allocation.

Received SDUs are logged as ``[CIS]`` or ``[BIS]`` lines.

Audio output
============

The received LC3 frames are decoded at 48 kHz and played on a TLV320 DAC over I2S.
Only one source (CIS or BIS) is played at a time; streams from the other source are ignored until the playing streams stop.

* The TLV320 is the I2S master, clocked from the I2S MCK of the nRF54L15.
* A jitter buffer per channel aligns the left and right BISes on their SDU timestamps.
* Playback starts once enough audio is buffered, and the drift between the audio source and the DAC clock is compensated by trimming the TLV320 PLL.
* The volume and mute state of the Volume Control Service, which the phone controls, set the DAC digital volume.

The output statistics are logged as ``OUT:`` and ``I2S:`` lines.

Bonding information is stored in the settings, so that a phone can reconnect after a reset.

Configuration
*************

|config|

Configuration options
=====================

Check and configure the following Kconfig options:

.. _CONFIG_NRF54L15_AUDIO_HEADSET_SCAN_SELF:

CONFIG_NRF54L15_AUDIO_HEADSET_SCAN_SELF - Scan for TMAP BMS without Broadcast Assistant
   Enables scanning for and syncing to a TMAP Broadcast Media Sender (BMS) without a Broadcast Assistant, in addition to accepting requests from a Broadcast Assistant.

Building and running
********************

.. |sample path| replace:: :file:`samples/bluetooth/nrf54l15_audio_headset`

.. include:: /includes/build_and_run.txt

Testing
=======

After programming the sample to your development kit, complete the following steps to test it with an Android phone:

1. Connect to the kit with an RTT viewer.
#. Reset the kit.
#. Pair the phone with the device named ``nRF54L15 Audio Headset``.
#. Play media or start a call on the phone to have the phone set up CIS.
   The audio is played on the DAC, and the received ISO data is logged as ``[CIS]`` lines.
#. Change the volume on the phone and observe that the output volume follows.
#. Start Audio Sharing, or add a broadcast source from the phone, to have the phone act as Broadcast Assistant.
   The audio is played on the DAC, and the received ISO data is logged as ``[BIS]`` lines.

Dependencies
************

This sample uses the following Zephyr libraries:

* :ref:`zephyr:bluetooth_api`
* :ref:`zephyr:bluetooth_audio`
