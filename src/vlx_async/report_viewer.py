"""Serve a saved shadow result without contacting the original service or API."""
import argparse
import base64
import json
import signal
import threading
from .dashboard import Dashboard
from .storage import confined

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory')
    parser.add_argument('--port',type=int,default=0)
    parser.add_argument('--listen',default='127.0.0.1')
    parser.add_argument('--allow-client',action='append',default=[])
    parser.add_argument('--duration',type=float,default=3600)
    args=parser.parse_args(argv)
    if args.listen not in {'127.0.0.1','localhost'} and not args.allow_client:
        parser.error('LAN listening requires --allow-client')
    run=confined(args.run_directory)
    state=json.loads((run/'latest.json').read_text())
    state['status']='saved_report'
    state['mode']='saved_report'
    state['report_notice']='已保存的影子运行报告；不再读取相机或调用 API'
    request_id=state.get('request_id')
    if isinstance(request_id,str) and request_id.isalnum():
        image=confined(run/('request_'+request_id+'.jpg'))
        if image.is_file():
            state.update(image_base64=base64.b64encode(image.read_bytes()).decode(),image_request_id=request_id)
    dashboard=Dashboard(args.listen,args.port,args.allow_client)
    dashboard.update(state)
    stop=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stop.set())
    print(json.dumps({'url':f'http://{args.listen}:{dashboard.port}/','run':str(run),'mode':'saved_report'}),flush=True)
    try:stop.wait(args.duration)
    finally:dashboard.close()

if __name__=='__main__':main()
