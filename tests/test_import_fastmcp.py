#!/usr/bin/env python3
"""
Minimal test to isolate FastMCP import performance
"""

import time

print("Starting import test...")
start_time = time.time()

print("Importing FastMCP...")
import_start = time.time()
from fastmcp import FastMCP
import_end = time.time()

print(f"FastMCP import took: {import_end - import_start:.2f} seconds")

print("Creating server instance...")
create_start = time.time()
mcp = FastMCP("test-server")
create_end = time.time()

print(f"Server creation took: {create_end - create_start:.2f} seconds")

total_time = time.time() - start_time
print(f"Total time: {total_time:.2f} seconds")

print("Ready to run server...")
