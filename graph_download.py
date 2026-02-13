import gzip
from urllib.request import urlretrieve

url = ("https://snap.stanford.edu/data/web-Google.txt.gz")
filename_gz = "/content/sample_data/web-Google.txt.gz"
filename_txt = "/content/sample_data/web-Google.txt" # This will be the decompressed file

urlretrieve(url, filename_gz)

# Decompress the .gz file and filter lines starting with '#'
with gzip.open(filename_gz, 'rb') as f_in:
    with open(filename_txt, 'wb') as f_out:
        # Skip the first 4 lines
        for _ in range(4):
            next(f_in) # Read and discard a line

        # Write the remaining lines to the output file
        for line_bytes in f_in:
            f_out.write(line_bytes)

print(f"Successfully decompressed and filtered {filename_gz} to {filename_txt}")



""" wikitalk"""



import gzip
from urllib.request import urlretrieve

url = ("https://snap.stanford.edu/data/wiki-Talk.txt.gz")
filename_gz = "/content/sample_data/wiki-Talk.txt.gz"
filename_txt = "/content/sample_data/wiki-Talk.txt" # This will be the decompressed file

urlretrieve(url, filename_gz)

# Decompress the .gz file and skip the first 4 lines
with gzip.open(filename_gz, 'rb') as f_in:
    with open(filename_txt, 'wb') as f_out:
        # Skip the first 4 lines
        for _ in range(4):
            next(f_in) # Read and discard a line

        # Write the remaining lines to the output file
        for line_bytes in f_in:
            f_out.write(line_bytes)

print(f"Successfully decompressed and filtered (skipped first 4 lines) {filename_gz} to {filename_txt}")




"""road fla"""

import gzip
from urllib.request import urlretrieve

url = "https://www.diag.uniroma1.it/challenge9/data/USA-road-d/USA-road-d.FLA.gr.gz"
filename_gz = "/content/sample_data/USA-road-d.FLA.gr.gz"
filename_gr = "/content/sample_data/USA-road-d.FLA.gr"

urlretrieve(url, filename_gz)

# Decompress the .gz file
with gzip.open(filename_gz, 'rb') as f_in:
    with open(filename_gr, 'wb') as f_out:
        f_out.write(f_in.read())

num_nodes = 0
num_edges = 0
edges = []

with open(filename_gr, 'r') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('c'):  # Skip empty lines and comment lines
            continue

        parts = line.split()
        line_type = parts[0]

        if line_type == 'p':
            # Problem line: p sp N M (N = nodes, M = edges)
            num_nodes = int(parts[2])
            num_edges = int(parts[3])
        elif line_type == 'a':
            # Arc line: a u v cost
            u = int(parts[1])
            v = int(parts[2])
            cost = int(parts[3])
            edges.append((u, v, cost))

print(f"Graph Information Summary:")
print(f"Number of nodes: {num_nodes}")
print(f"Number of edges: {num_edges}")
print(f"First 5 edges: {edges[:5]}")
print(f"Last 5 edges: {edges[-5:]}")
