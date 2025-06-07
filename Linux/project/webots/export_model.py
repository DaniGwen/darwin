import torch
import torch.nn as nn

# IMPORTANT: Define the same model architecture you used for training.
# The input_size and output_size must match.
class DarwinNet(nn.Module):
    def __init__(self):
        super(DarwinNet, self).__init__()
        # Example architecture: 23 inputs -> 128 neurons -> 128 -> 20 outputs
        self.layer1 = nn.Linear(23, 128)
        self.layer2 = nn.Linear(128, 128)
        self.layer3 = nn.Linear(128, 20)

    def forward(self, x):
        x = torch.relu(self.layer1(x))
        x = torch.relu(self.layer2(x))
        x = self.layer3(x)
        return x

# 1. Instantiate your model
model = DarwinNet()

# 2. Load the weights from your saved training file
#    (Replace "my_trained_model.pth" with your actual file name)
model.load_state_dict(torch.load("my_trained_model.pth"))

# 3. Set the model to evaluation mode (very important!)
model.eval()

# 4. Create a dummy input tensor with the correct shape for the exporter to trace
#    (Batch size of 1, 23 input features)
dummy_input = torch.randn(1, 23)

# 5. Export the model to the ONNX format
torch.onnx.export(model,
                  dummy_input,
                  "darwin_model.onnx", # This is the output file name
                  input_names=['input'],
                  output_names=['output'])

print("Successfully exported the model to darwin_model.onnx")