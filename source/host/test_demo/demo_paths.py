from pathlib import Path
from datetime import datetime
def default_log_directory(script):
 return str(Path(__file__).resolve().parent/"logs"/(Path(script).stem+"_"+datetime.now().strftime("%Y%m%d_%H%M%S_%f")))
