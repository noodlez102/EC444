import os

def clean_folder(image_dir, csv_dir):
    image_files = set(os.listdir(image_dir))
    csv_files = set(os.listdir(csv_dir))
    
    # Find image files without CSV counterparts
    orphaned_image_files = image_files - csv_files
    
    # Remove orphaned image files
    for filename in orphaned_image_files:
        file_path = os.path.join(image_dir, filename)
        os.remove(file_path)
        print(f"Removed orphaned image file: {filename}")

# Example usage:
image_dir = 'data_folder2'
csv_dir = 'data_folder2'
clean_folder(image_dir, csv_dir)
