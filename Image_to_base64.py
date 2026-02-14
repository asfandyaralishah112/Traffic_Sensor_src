import base64
import os
import sys

def image_to_base64(image_path):
    if not os.path.isfile(image_path):
        print("File not found.")
        return

    # Read image as binary
    with open(image_path, "rb") as img_file:
        encoded_bytes = base64.b64encode(img_file.read())

    encoded_string = encoded_bytes.decode("utf-8")

    # Output file name (same name + .txt)
    base_name = os.path.splitext(image_path)[0]
    output_file = base_name + ".txt"

    with open(output_file, "w") as f:
        f.write(encoded_string)

    print(f"Base64 saved to: {output_file}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage:")
        print("python image_to_base64.py logo.png")
    else:
        image_to_base64(sys.argv[1])
