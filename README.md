
# Bunraku

**Bunraku** is an ongoing project by [Amir Bastan](https://amirbastan.com/) exploring robot control, real-time simulation, and collaborative interaction in creative and artistic contexts. A visual live programming tool, developed for artists, designers, and creative coders, it provides an experimental space for working with robotic movement outside of purely industrial applications.

Rather than treating robots as rigid tools for automation, Bunraku approaches them as dynamic, responsive, and performative entities. The framework allows for live interaction, networked collaboration, and alternative input methods, making it particularly suited for multimedia installations, performances, and research into non-utilitarian robotics.

**Bunraku** (pronounced **boon-rah-koo**) is a form of traditional Japanese puppet theatre that dates back to the 17th century. It is known for its highly intricate puppets, which are operated by multiple puppeteers working together to create fluid, lifelike movements. The performances often feature synchronized storytelling, combining narration, music, and precise puppet manipulation.

---

## Technical Foundations

- Developed in VL, the visual programming language of [vvvv gamma](http://vvvv.org), supporting a flexible and modular workflow.  
- Built on, and deeply inspired by [KUKA|prc](https://robotsinarchitecture.org/), while focusing on interactive and artistic performativity.

## Core Characteristics

- Real-time remote collaboration, allowing multiple users to work within the same simulation environment.  
- Currently supports KUKA industrial robots and Boston Dynamics’ Spot, with potential for further expansion.  
- Integrates with various communication protocols, making it adaptable to different workflows.  
- Any input device that connects to a Windows PC, from game controllers to motion sensors, can influence robotic behaviour.  
- Developed through direct engagement with artistic and interactive projects, shaping its evolving capabilities.

## Current Status

Bunraku development is ongoing, shaped by practical use cases and evolving ideas about robotic agency, interaction, and performativity.  
It is used in-house as an experimental tool and has contributed to the development of various robotic art installations and performances.

> If you use Bunraku to conduct research, we ask that you cite the software as a reference:

```bash
 @misc{bastan_bunraku_2023,
	address = {Linz, Austria},
	title = {Bunraku},
	url = {https://github.com/baxtan/VL.Bunraku},
	abstract = {Bunraku (pronounced boon-rah-koo) is an ongoing project by Amir Bastan exploring robot control, real-time simulation, and collaborative interaction in creative and artistic contexts. A visual live programming tool, developed for artists, designers, and creative coders, it provides an experimental space for working with robotic movement outside of purely industrial applications.},
	author = {Bastan, Amir},
	month = apr,
	year = {2023},
}
```


## Disclaimer

Bunraku is provided "as is", without warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular purpose and noninfringement. In no event shall the authors or copyright holders be liable for any claim, damages or other liability, whether in an action of contract, tort or otherwise, arising from, out of or in connection with the software or the use or other dealings in the software.

---

## Git LFS Notice

> **Note**: This repository uses Git LFS for managing large files (e.g., `.glb` models).  
> If you're cloning the repository, please run:

```bash
git lfs install
git lfs pull


