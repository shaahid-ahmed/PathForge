# PathForge

A Python-based tool to convert 2D DXF drawings into optimized G-code for CNC machines. Supports key geometric entities with features like kerf compensation, entity sorting, and basic path optimization.

![Screenshot](<img width="1918" height="1017" alt="image" src="https://github.com/user-attachments/assets/60431841-e919-4d5b-9088-446b3a82e8cc" />
) <!-- Replace with actual screenshot if GUI exists -->

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
