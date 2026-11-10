import gello_readonly_entry as entry
entry.SECONDS=5.0
if __name__=='__main__':
    result=entry.run(force_feedback=True)
    raise SystemExit(0 if result['status']=='completed' else 1)
