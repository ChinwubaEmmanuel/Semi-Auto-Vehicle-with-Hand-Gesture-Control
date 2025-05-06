import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
import joblib

# Get data set for training
data = pd.read_csv('hand_gesture_dataset_2.csv')
x = data.drop('label', axis=1)
y = data['label']

# Split between train and test data then perform ops
x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.2, random_state=42) # 20% to test, other for training
model = RandomForestClassifier(n_estimators=100)
model.fit(x_train, y_train)

accuracy = model.score(x_test, y_test)
print(f"Accuracy: {accuracy * 100:.2f}%") # disp model accuracy

joblib.dump(model, 'gesture_model_2.pkl')
