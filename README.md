# PathForge

A Python-based tool to convert 2D DXF drawings into optimized G-code for CNC machines. Supports key geometric entities with features like kerf compensation, entity sorting, and basic path optimization.

![Screenshot](https://github.com/shaahid-ahmed/PathForge/blob/main/src1.png) <!-- Replace with actual screenshot if GUI exists -->

---

## Features

- ✅ Support for LINE, ARC, CIRCLE, and SPLINE entities  
- ✂️ Kerf compensation  
- 📈 Entity sorting for reduced travel distance  
- 🔁 Basic path optimization  
- 📤 Export clean G-code compatible with most CNC controllers  
- ⚙️ Easily extendable architecture for custom post-processing or entity support

---

## Installation

Clone the repository and install dependencies:

```bash
git clone https://github.com/shaahid-ahmed/PathForge.git
cd PathForge
pip install -r requirements.txt
