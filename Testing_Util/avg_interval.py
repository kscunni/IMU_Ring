import pandas as pd

def process_csv_increases(file_path):
    # Load the CSV file
    df = pd.read_csv(file_path)
    
    # Select the first column using iloc (index location 0)
    first_col = df.iloc[:, 0]
    
    # 1. Get the increase compared to the number in the previous row
    # The .diff() function computes the difference between the current and previous row
    increases = first_col.diff()
    
    # 2. Isolate only the values greater than 10
    significant_increases = increases[increases > 10]
    
    # 3. Get the average of these non-negligible increase values
    # Check if there are any values greater than 10 to avoid dividing by zero
    if not significant_increases.empty:
        average_increase = significant_increases.mean()
        print(f"The average of the increases greater than 10 is: {average_increase:.2f}")
        return average_increase
    else:
        print("No increase values were greater than 10.")
        return None

# Example usage:
# Replace 'your_file.csv' with the actual path to your CSV
process_csv_increases('imu_data.csv')