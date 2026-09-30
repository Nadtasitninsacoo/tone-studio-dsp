# Steinberg ASIO SDK — the three headers JUCE needs

`common/iasiodrv.h`, `common/asio.h` and `common/asiosys.h`, copied unmodified from
**ASIO SDK 2.3.4 (2025-10-15)**, downloaded from
<https://www.steinberg.net/asiosdk>.

Steinberg dual-licenses this SDK: its own proprietary licence, or the **GNU GPL v3**.
This project is GPL v3 (see `/LICENSE`), so these files are used **under the GPL v3
option**. `LICENSE.txt` is Steinberg's text stating both options, kept beside the files as
the licence requires.

Only these three are here because they are all `juce_audio_devices` includes
(`#include <iasiodrv.h>`, which pulls in the other two). Nothing else from the SDK is
compiled.

ASIO is a trademark of Steinberg Media Technologies GmbH. No ASIO logo is used anywhere in
this project; using it needs Steinberg's separate agreement.
