> [!WARNING]
> If you have paid for PlayStation VR2 Toolkit, you have been scammed and you should immediately request a refund. PlayStation VR2 Toolkit is entirely free and **is intended for NON-COMMERCIAL use only**, we do not attempt to profit off of it. Additionally, eye tracking data from PlayStation VR2 Toolkit **must not** be used in commercial environments and/or for commercial purposes. \
> \
> PlayStation VR2 Toolkit is provided "as is", without warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular purpose and noninfringement. In no event shall the authors or copyright holders be liable for any claim, damages or other liability, whether in an action of contract, tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in the software.

> [!NOTE]
> Bnuuy Solutions and PlayStation VR2 Toolkit is not affiliated with Sony Interactive Entertainment, our project is not official or endorsed by Sony Interactive Entertainment in any way. \
> \
> Sony, Sony Interactive Entertainment and PS VR2/PlayStation VR2 are trademarks or registered trademarks of Sony Interactive Entertainment LLC in the United States of America and elsewhere.

<p align="center"><img src="https://github.com/BnuuySolutions/PSVR2Toolkit/blob/main/assets/Icon.png?raw=true" width="128" height="128"></p>
<h1 align="center">PlayStation VR2 Toolkit</h1>

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Unofficial modification for the official PlayStation VR2 driver/app, which aims to improve your PlayStation VR2 experience on PC.

## Features
- Eye tracking\*
- Improved controller prediction
- Improved controller haptics (w/ PCM haptics)
- Adaptive triggers

For developers, we also have our own API library, which allows you to take full advantage of these features.

\* Eye tracking calibration is currently not available yet, we have a few things to work out before we are ready to ship this feature!

## Installation Guide
1.) Open Steam, go to the PS VR2 app, click on the cog wheel, and go to "Manage -> Browse local files". (If you are using a copy of the PS VR2 app not installed by Steam, go to that instead.)

2.) Inside the newly opened file explorer, go into "SteamVR_Plug-In", then "bin" and finally "win64".

3.) Rename "driver_playstation_vr2.dll" to "driver_playstation_vr2_orig.dll" (**IT MUST BE CALLED "driver_playstation_vr2_orig.dll", DO NOT RENAME IT TO ANYTHING ELSE, IT MUST BE EXACTLY THAT**)

4.) Download the "driver_playstation_vr2.dll" attached in a release, and copy/move it into the same folder where "driver_playstation_vr2_orig.dll" is at.

5.) Your "win64" directory should now have 2 DLL files inside it, "driver_playstation_vr2.dll" and "driver_playstation_vr2_orig.dll". If you do not have both of those files, you fucked something up.

6.) Enjoy your new features, please give us feedback in our [Discord](https://discord.gg/jym8CgJPF3).

# Contact
Have any legal complaints or questions?
- Email: `wdh at bnuuy.solutions`
- Discord: `notahopper`
