
# 🤖 Pre-Trained ACT Policy Weights

Due to GitHub's strict 100MB file size limit, the trained model weights (`model.safetensors`, ~197MB) are not stored in this repository. Instead, they are hosted securely on the Hugging Face Model Hub.

## How to Download the Weights

You can download the exact pre-trained policy needed for this project directly from Hugging Face.

### Option 1: Using the Hugging Face CLI (Recommended)
If you have the `huggingface_hub` python package installed, simply run this command from **this directory** (`trained_policy/`):

\`\`\`bash
huggingface-cli download Anany444/Drawing-Robotic-Arm-ACT-Policy --local-dir ACT --local-dir-use-symlinks False
\`\`\`
*This will automatically create the `ACT` folder and place the model weights exactly where the `policy_executor.py` script expects them.*

### Option 2: Manual Download
If you prefer to download them manually:
1. Go to the Hugging Face Repository: [Anany444/Drawing-Robotic-Arm-ACT-Policy](https://huggingface.co/Anany444/Drawing-Robotic-Arm-ACT-Policy/tree/main)
2. Download all the files (including `model.safetensors`, `config.json`, and the processor JSONs).
3. Create a folder structure in this directory so it looks exactly like this:
   \`\`\`text
   trained_policy/
   └── ACT/
       ├── config.json
       ├── model.safetensors
       ├── policy_postprocessor.json
       └── ... (other files)
   \`\`\`

Once the model files are present, you can run the `policy_executor` node successfully!
