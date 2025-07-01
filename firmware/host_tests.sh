#!/usr/bin/env bash

# This script is used to run host_test unit tests. These are tests that run on the host machine.

# Usage: ./host_tests.sh [(optional) component] [(optional) test_name]

# Example: ./host_tests.sh platform
# Example: ./host_tests.sh platform test_read_lidar
# Example: ./host_tests.sh

# If no arguments are provided, run all tests
# If a component is provided, run all tests for that component
# If a component and test name are provided, run that specific test

set -e

passes=0
failures=0

root_dir=$(dirname "$(realpath "$0")")

while IFS= read -r test_dir; do
  component=$(basename "$(dirname "$test_dir")")
  echo "=== Building $component host tests ==="
  pushd "$test_dir" > /dev/null
  IDF_TARGET=linux idf.py build
  echo "=== Running $component host tests ==="
  if ./build/*.elf; then
    echo "=== Host tests for $component passed ==="
    passes=$((passes + 1))
  else
    echo "=== Host tests for $component failed ==="
    failures=$((failures + 1))
  fi
  popd > /dev/null
done < <(find "$root_dir/components" -type d -path '*/host_test')

echo "=========== Summary ==========="
echo "Total tests passed: $passes"
echo "Total tests failed: $failures"

exit $failures
