import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
from pathlib import Path

def main():
    rclpy.init()
    node = Node('manual_trainer')
    
    # Import the data manager and detector
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from ai_core.self_learning_classification import SelfLearningDataManager, OnlineLearningWeaponDetector
    
    data_manager = SelfLearningDataManager()
    weapon_detector = OnlineLearningWeaponDetector()
    
    print("=" * 60)
    print(" Manual Model Retraining")
    print("=" * 60)
    
    sample_counts = data_manager.get_sample_counts()
    print("\n Available Training Samples:")
    for class_name, count in sample_counts.items():
        print(f"   {class_name}: {count} samples")
    
    total_samples = sum(sample_counts.values())
    print(f"\n   Total: {total_samples} samples")
    
    if total_samples < 10:
        print("\n Not enough samples for training!")
        return
    
    response = input("\n Start retraining? (yes/no): ")
    
    if response.lower() == 'yes':
        print("\nStarting retraining...")
        success = weapon_detector.incremental_train(
            data_manager,
            num_epochs=5,
            batch_size=4
        )
        
        if success:
            print(f"\n✓ Retraining complete! Model v{weapon_detector.current_version}")
        else:
            print("\n Retraining failed!")
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
