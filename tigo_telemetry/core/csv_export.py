import csv
from dataclasses import fields
from typing import Iterable
from core.models import CsvRow


def _format_val(val: object) -> str:
    if val is None:
        return ""
    if isinstance(val, float):
        return f"{val:.5f}"
    return str(val)


def export_rows_to_csv(file_path: str, rows: Iterable[CsvRow]) -> int:
    """Grava as amostras de telemetria em um arquivo CSV e retorna a quantidade de linhas."""
    row_list = list(rows)
    if not row_list:
        return 0

    headers = [f.name for f in fields(CsvRow)]
    with open(file_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(headers)
        for row in row_list:
            writer.writerow([_format_val(getattr(row, col)) for col in headers])
    return len(row_list)
