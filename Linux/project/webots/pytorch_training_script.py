import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import numpy as np

# --- 1. Define the Neural Network Architecture ---
# This MUST match the architecture you use for export and in C++.
class DarwinNet(nn.Module):
    def __init__(self, input_size=23, output_size=20):
        super(DarwinNet, self).__init__()
        # Simple three-layer network
        self.layer1 = nn.Linear(input_size, 128)
        self.layer2 = nn.Linear(128, 128)
        self.layer3 = nn.Linear(128, output_size)

    def forward(self, x):
        x = torch.relu(self.layer1(x))
        x = torch.relu(self.layer2(x))
        # No activation on the last layer for regression tasks
        x = self.layer3(x)
        return x

# --- 2. Create a Custom PyTorch Dataset ---
# This class will load the training_data.csv file you generated.
class RobotPoseDataset(Dataset):
    def __init__(self, csv_file):
        # For this example, we'll generate random data.
        # In a real project, you would load your CSV here using a library like pandas.
        # E.g., data = pd.read_csv(csv_file)
        print("Generating dummy data... In a real project, load from CSV here.")
        num_samples = 5000
        input_size = 23
        output_size = 20
        
        # Simulating states where the robot is slightly off from zero
        self.x_data = np.random.randn(num_samples, input_size).astype(np.float32) * 0.5 
        
        # The "correct" output is always the target pose (all zeros)
        self.y_data = np.zeros((num_samples, output_size), dtype=np.float32)

    def __len__(self):
        return len(self.y_data)

    def __getitem__(self, idx):
        # Return one sample (input, output)
        inputs = torch.from_numpy(self.x_data[idx])
        outputs = torch.from_numpy(self.y_data[idx])
        return inputs, outputs

# --- 3. The Main Training Script ---
if __name__ == '__main__':
    # Hyperparameters
    input_size = 23
    output_size = 20
    learning_rate = 0.001
    batch_size = 64
    num_epochs = 50

    # Initialize Dataset and DataLoader
    dataset = RobotPoseDataset(csv_file='training_data.csv') # Pass your data file
    train_loader = DataLoader(dataset=dataset, batch_size=batch_size, shuffle=True)

    # Initialize the model
    model = DarwinNet(input_size, output_size)
    print("Model Architecture:")
    print(model)

    # Loss function and optimizer
    # Mean Squared Error is good for regression tasks (predicting positions)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)

    # --- The Training Loop ---
    print("\nStarting training...")
    for epoch in range(num_epochs):
        for i, (inputs, targets) in enumerate(train_loader):
            # Forward pass: compute predicted y by passing x to the model.
            outputs = model(inputs)

            # Compute loss
            loss = criterion(outputs, targets)

            # Backward pass and optimization
            optimizer.zero_grad()  # Clear gradients from previous iteration
            loss.backward()        # Compute gradients
            optimizer.step()       # Update weights

        # Print progress
        print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.6f}')

    print("\nTraining complete!")

    # --- 4. Save the Trained Model ---
    # This creates the file you will need for the ONNX export step.
    final_model_path = 'my_trained_model.pth'
    torch.save(model.state_dict(), final_model_path)
    print(f"Trained model weights saved to {final_model_path}")

    print("\nNext steps:")
    print("1. Run your 'export_model.py' script to create 'darwin_model.onnx'.")
    print("2. Copy 'darwin_model.onnx' to your Raspberry Pi.")