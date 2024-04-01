# WPAN SDK

## Overview

The WPAN SDK is a repository provided by Infineon that contains PAL, HAL, and
PLD submodules or implementations for OpenThread and Matter development. This
repository is used by developers for internal development before going public
to the wpan-sdk repository.

## Hardware Abstraction Layer

The Hardware Abstraction Layer (HAL) is a high-level interface that provides a
generic interface for the upper layer across multiple product families. It
enables developers to configure and use hardware blocks with ease.

## Platform Abstraction Layer

The Platform Abstraction Layer (PAL) serves as the middle layer between
application libraries and the underlying platform. It provides a standardized
interface that allows application libraries to access platform-specific
functionalities without needing to interact with the platform's APIs or
implementation details directly.

## Peripheral Driver Library

The Peripheral Driver Library (PLD) integrates device header files and
peripheral drivers into a single directory. The drivers abstract the hardware
functions into a set of easy-to-use APIs.
