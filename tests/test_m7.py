import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from numba import get_num_threads, set_num_threads, config

from navgen.areas import build_areas
from navgen.collision import batch_trace
from navgen.hiding import compute_hiding_spots
from navgen.navio import write_nav
from navgen.runtime import configure_threads, source_fingerprint, trace_count
from navgen.sampling import sample
from test_generation import collision, quiet


class M7Tests(unittest.TestCase):
    def test_thread_determinism_and_trace_counters(self):
        old=get_num_threads();reference=None;sampled=None
        try:
            for threads in sorted({1,min(4,config.NUMBA_NUM_THREADS),config.NUMBA_NUM_THREADS}):
                set_num_threads(threads)
                world=collision([([0,0,-20],[800,600,0]),([100,0,0],[150,175,12]),
                                 ([200,150,60],[400,300,80]),([0,275,0],[150,300,100])])
                arrays,index=sample(world,np.array([[50.,50.,1.]]),[[0,0,0],[800,600,100]],quiet)
                if threads>1:self.assertGreater(world.sampling_speculation['cache_hits'],0)
                node_bytes=[a.tobytes() for a in arrays]
                if sampled is None:sampled=node_bytes
                else:self.assertEqual(node_bytes,sampled)
                graph,_=build_areas(world,arrays,index,quiet);nav=graph.nav(1)
                compute_hiding_spots(nav,world);raw=write_nav(nav)
                if reference is None:reference=raw
                else:self.assertEqual(raw,reference)
                n=1001;start=np.tile([50.,50.,100.],(n,1));end=start-[0,0,200]
                zero=np.zeros((n,3));before=trace_count(world.scene)
                hits=batch_trace(world.scene,start,end,zero,zero)
                self.assertEqual(trace_count(world.scene)-before,n)
                np.testing.assert_array_equal(hits,np.tile(hits[0],(n,1)))
        finally:set_num_threads(old)

    def test_cache_namespace_detects_imported_helper_change(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'caller.py').write_text('from helper import VALUE\n')
            helper=root/'helper.py';helper.write_text('VALUE=1\n');first=source_fingerprint(root)
            helper.write_text('VALUE=2\n');self.assertNotEqual(first,source_fingerprint(root))
            helper.write_text('VALUE=1\n');self.assertEqual(first,source_fingerprint(root))

    def test_thread_limit_validation(self):
        old=get_num_threads()
        try:
            self.assertEqual(configure_threads(1),1)
            for value in (0,-1,1.5,True,config.NUMBA_NUM_THREADS+1):
                with self.assertRaises(ValueError):configure_threads(value)
        finally:set_num_threads(old)

    def test_fresh_process_cached_trace_initializes_threads(self):
        code='''import json,sys
sys.path.insert(0,sys.argv[1])
import numpy as np
from navgen.collision import SceneBuilder,trace
from navgen.runtime import trace_count
scene=SceneBuilder().pack();zero=np.zeros(3)
hit=trace(scene,zero,np.ones(3),zero,zero)
print(json.dumps([hit[0],trace_count(scene)]))
'''
        with tempfile.TemporaryDirectory() as folder:
            env=dict(os.environ);env['NUMBA_CACHE_DIR']=folder;env['NUMBA_DEBUG_CACHE']='1'
            command=[sys.executable,'-I','-B','-X','faulthandler','-c',code,str(Path(__file__).resolve().parents[1])]
            for attempt in range(2):
                output=subprocess.check_output(command,env=env,text=True,stderr=subprocess.STDOUT)
                self.assertEqual(output.splitlines()[-1],'[1.0, 1]')
                if attempt:self.assertIn('[cache] data loaded',output)


if __name__=='__main__':unittest.main()
