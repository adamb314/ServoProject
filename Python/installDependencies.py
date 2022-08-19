#!/bin/python3

'''
Python script for installing dependencies
'''

from ServoProjectModules import DependencyHandler

if __name__ == '__main__':
    DependencyHandler.__init__(automaticInstall=True)
