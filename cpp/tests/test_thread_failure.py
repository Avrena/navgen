"""Linux regression test for partially constructed native worker pools.

    python cpp/tests/test_thread_failure.py \
        --test-binary cpp/build/navgen_sampling_areas_tests

An interposed pthread_create returns EAGAIN on the second attempt. Each native
test verifies the first worker was joined before the error propagated.
"""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile


INTERPOSER = r'''
#define _GNU_SOURCE
#include <pthread.h>
#include <dlfcn.h>
#include <errno.h>
static int calls=0;
int pthread_create(pthread_t* thread,const pthread_attr_t* attr,void*(*start)(void*),void* arg) {
    int(*real_create)(pthread_t*,const pthread_attr_t*,void*(*)(void*),void*)=dlsym(RTLD_NEXT,"pthread_create");
    if(++calls==2)return EAGAIN;
    return real_create(thread,attr,start,arg);
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--test-binary', type=Path, required=True)
    parser.add_argument('--cc', default=os.environ.get('CC', 'cc'))
    args = parser.parse_args()
    if sys.platform != 'linux' or (os.cpu_count() or 1) < 3:
        print('SKIP: requires Linux and at least three logical cores')
        return 0
    import resource

    def no_core_dump():
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

    with tempfile.TemporaryDirectory(prefix='navgen-thread-failure-') as temporary:
        source = Path(temporary) / 'fail_thread.c'
        library = Path(temporary) / 'fail_thread.so'
        source.write_text(INTERPOSER)
        subprocess.run([*shlex.split(args.cc), '-shared', '-fPIC', str(source),
                        '-ldl', '-o', str(library)], check=True)
        env = dict(os.environ)
        env['LD_PRELOAD'] = str(library)
        for helper in ('sampling', 'areas', 'generic'):
            process = subprocess.run([str(args.test_binary.resolve()), '--thread-failure', helper],
                                     env=env, text=True, capture_output=True,
                                     preexec_fn=no_core_dump)
            if process.returncode or 'thread construction failure joined workers' not in process.stdout:
                raise AssertionError(f'{helper}: exit {process.returncode}\n{process.stdout}{process.stderr}')
            print(f'MATCH {helper}: construction error propagated after joining worker', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
