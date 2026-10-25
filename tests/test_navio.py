import unittest

from navgen.bsp import DEFAULT_GAME
from navgen.navio import read_nav,write_nav


class NavRoundTripTests(unittest.TestCase):
    def test_engine_files(self):
        for name in ('gm_construct','gm_flatgrass'):
            with self.subTest(map=name):
                raw=(DEFAULT_GAME/'maps'/f'{name}.nav').read_bytes()
                parsed=read_nav(raw)
                self.assertEqual(parsed['version'],16)
                self.assertEqual(parsed['subversion'],0)
                self.assertEqual(raw,write_nav(parsed))


if __name__=='__main__':unittest.main()
