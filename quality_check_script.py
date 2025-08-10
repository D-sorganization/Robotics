#!/usr/bin/env python3
"""Quality check script to verify AI-generated code meets standards."""

import ast
import re
import sys
from pathlib import Path

# Configuration
BANNED_PATTERNS = [
    (re.compile(r"\bTODO\b"), "TODO placeholder found"),
    (re.compile(r"\bFIXME\b"), "FIXME placeholder found"),
    (re.compile(r"^\s*\.\.\.\s*$"), "Ellipsis placeholder"),
    (re.compile(r"NotImplementedError"), "NotImplementedError placeholder"),
    (re.compile(r"<.*>"), "Angle bracket placeholder"),
    (re.compile(r"your.*here", re.IGNORECASE), "Template placeholder"),
    (re.compile(r"insert.*here", re.IGNORECASE), "Template placeholder"),
]

# More intelligent pass statement detection
PASS_PATTERNS = [
    # Empty pass statements that are likely placeholders
    (re.compile(r"^\s*pass\s*$"), "Empty pass statement"),
    # Pass statements in empty blocks that might be placeholders
    (re.compile(r"^\s*if\s+.*:\s*$"), "Empty if block - consider adding logic or comment"),
    (re.compile(r"^\s*else:\s*$"), "Empty else block - consider adding logic or comment"),
    (re.compile(r"^\s*except\s+.*:\s*$"), "Empty except block - consider adding error handling"),
]

MAGIC_NUMBERS = [
    (re.compile(r"(?<![0-9])3\.141(?!59)"), "Use math.pi instead of 3.141"),
    (re.compile(r"(?<![0-9])9\.8[0-9]?(?![0-9])"), "Define GRAVITY_M_S2 constant"),
    (re.compile(r"(?<![0-9])6\.67[0-9]?(?![0-9])"), "Define gravitational constant"),
]


def _is_in_class_definition(lines: list[str], line_num: int) -> bool:
    """Check if the pass statement is in a class definition."""
    for i in range(line_num - 1, max(0, line_num - 10), -1):
        prev_line = lines[i - 1].strip()
        if prev_line.startswith("class "):
            return True
        if prev_line.startswith("def "):
            return False
    return False


def _is_in_control_structure(lines: list[str], line_num: int) -> bool:
    """Check if the pass statement is in a control structure."""
    for i in range(line_num - 1, max(0, line_num - 5), -1):
        prev_line = lines[i - 1].strip()
        if prev_line.endswith(":") and any(
            keyword in prev_line
            for keyword in ["try:", "except", "finally:", "with ", "if __name__"]
        ):
            return True
    return False


def _is_in_try_except_block(lines: list[str], line_num: int) -> bool:
    """Check if the pass statement is in a try/except block."""
    for i in range(line_num - 1, max(0, line_num - 5), -1):
        prev_line = lines[i - 1].strip()
        if "try:" in prev_line or "except" in prev_line:
            return True
    return False


def _is_in_context_manager(lines: list[str], line_num: int) -> bool:
    """Check if the pass statement is in a context manager."""
    for i in range(line_num - 1, max(0, line_num - 3), -1):
        prev_line = lines[i - 1].strip()
        if prev_line.startswith("with "):
            return True
    return False


def is_legitimate_pass_context(lines: list[str], line_num: int) -> bool:
    """Check if a pass statement is in a legitimate context."""
    if line_num <= 0 or line_num > len(lines):
        return False

    line = lines[line_num - 1].strip()
    if line != "pass":
        return False

    # Check various legitimate contexts
    if _is_in_class_definition(lines, line_num):
        return True
    if _is_in_control_structure(lines, line_num):
        return True
    if _is_in_try_except_block(lines, line_num):
        return True
    if _is_in_context_manager(lines, line_num):
        return True

    return False


def check_banned_patterns(
    lines: list[str],
    filepath: Path,
) -> list[tuple[int, str, str]]:
    """Check for banned patterns in lines."""
    issues: list[tuple[int, str, str]] = []
    # Skip checking this file for its own patterns
    if filepath.name == "quality_check_script.py":
        return issues

    for line_num, line in enumerate(lines, 1):
        # Check for basic banned patterns
        for pattern, message in BANNED_PATTERNS:
            if pattern.search(line):
                issues.append((line_num, message, line.strip()))

        # Special handling for pass statements
        if re.match(r"^\s*pass\s*$", line) and not is_legitimate_pass_context(
            lines, line_num
        ):
            issues.append(
                (line_num, "Empty pass statement - consider adding logic or comment", line.strip())
            )

    return issues


def check_magic_numbers(
    lines: list[str],
    filepath: Path,
) -> list[tuple[int, str, str]]:
    """Check for magic numbers in lines."""
    issues: list[tuple[int, str, str]] = []

    # Skip checking this file for magic numbers
    if filepath.name == "quality_check_script.py":
        return issues

    for line_num, line in enumerate(lines, 1):
        for pattern, message in MAGIC_NUMBERS:
            if pattern.search(line):
                issues.append((line_num, message, line.strip()))

    return issues


def check_ast_issues(content: str) -> list[tuple[int, str, str]]:
    """Check for AST-level issues."""
    issues: list[tuple[int, str, str]] = []

    try:
        tree = ast.parse(content)
        for node in ast.walk(tree):
            if isinstance(node, ast.Pass):
                # This is a basic check - more sophisticated logic is in check_banned_patterns
                pass
    except SyntaxError as e:
        issues.append((e.lineno or 0, f"Syntax error: {e.msg}", ""))

    return issues


def check_file(filepath: Path) -> list[tuple[int, str, str]]:
    """Check a single file for quality issues."""
    try:
        with open(filepath, encoding="utf-8") as f:
            lines = f.readlines()

        issues = []
        issues.extend(check_banned_patterns(lines, filepath))
        issues.extend(check_magic_numbers(lines, filepath))
        issues.extend(check_ast_issues("".join(lines)))

        return issues
    except Exception as e:
        return [(0, f"Error reading file: {e}", "")]


def main() -> None:
    """Main function to run quality checks."""
    python_files = list(Path().rglob("*.py"))

    # Exclude directories as requested by user
    exclude_dirs = {
        "matlab",  # Exclude all MATLAB code
        ".git",
        "__pycache__",
        ".ruff_cache",
        ".mypy_cache",
        ".ipynb_checkpoints",
    }

    python_files = [f for f in python_files if not any(
        part in exclude_dirs for part in f.parts
    )]

    total_issues = 0
    files_with_issues = 0

    for filepath in python_files:
        issues = check_file(filepath)
        if issues:
            files_with_issues += 1
            print(f"\n{filepath}:")
            for line_num, message, line in issues:
                print(f"  Line {line_num}: {message}")
                if line:
                    print(f"    {line}")
            total_issues += len(issues)

    if total_issues == 0:
        print("✅ Quality check PASSED")
        print(f"Checked {len(python_files)} Python files")
    else:
        print("\n❌ Quality check FAILED")
        print(f"Found {total_issues} issues in {files_with_issues} files")
        sys.exit(1)


if __name__ == "__main__":
    main()
