import csv
import sys
import os

def hex_cycles_to_seconds(input_csv, output_csv, clock_mhz):
  clock_hz = clock_mhz * 1_000_000
  with open(input_csv, 'r') as infile, open(output_csv, 'w', newline='') as outfile:
    reader = csv.reader(infile)
    writer = csv.writer(outfile)
    writer.writerow(['seconds'])
    for row in reader:
      if not row or not row[0].startswith('0x'):
        continue
      cycles = int(row[0], 16)
      seconds = cycles / clock_hz
      writer.writerow([f"{seconds:.9f}"])

if __name__ == "__main__":
  if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} <input_csv>")
    sys.exit(1)
  input_csv = sys.argv[1]
  if not os.path.isfile(input_csv):
    print("Input file does not exist.")
    sys.exit(1)
  try:
    clock_mhz = float(input("Enter clock frequency in MHz: ").strip())
  except ValueError:
    print("Invalid clock frequency.")
    sys.exit(1)
  output_csv = os.path.splitext(input_csv)[0] + "_seconds.csv"
  hex_cycles_to_seconds(input_csv, output_csv, clock_mhz)
  print(f"Output written to {output_csv}")
