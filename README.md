# Physics_Engine_Optimisation
Accompanying code to the publication "Traversing the Reality Gap via Simulator Tuning". Most of this code was written pre-2020 and as such has not been tested recently and is not maintained. If you are having issues running with it please get in contact. 

*[Jack Collins](https://jacktcollins.com), [Ross Brown](https://staff.qut.edu.au/staff/r.brown), [Jürgen Leitner](http://juxi.net) and [David Howard](https://people.csiro.au/H/D/David-Howard)

## Publication

**Traversing the Reality Gap via Simulator Tuning**

Australasian Conference on Robotics and Automation (ACRA) Australian Robotics and Automation Association (ARAA), 2021

[arXiv](https://arxiv.org/pdf/2003.01369.pdf)

<!-- If you use this work, please cite the following as appropriate:

```text
@inproceedings{"Collins2021TraversingTuning", 
	title={{Traversing the Reality Gap via Simulator Tuning}}, 
	author={Jack Collins, Ross Brown, J\"urgen Leitner and David Howard}, 
	booktitle={2021 Australasian Conference on Robotics and Automation (ACRA)}
	year={2021} 
}

``` -->


## Installation

This code was developed with Python 3.5 on Ubuntu 16.04. Python libraries required: numpy, scipy optimize, matplotlib, sklearn and skopt.

V-REP: Version 3.5.0*

PyBullet: Version 2.1.1

*Please add vrep.py, vrepConst.py and remoteApi.so files relevant for your distribution of V-Rep to the Software directory.

## Running

To execute run main.py

There are three arguments.

1. Physics Engine: A string. The choice is between 'PyBullet', 'Bullet278', 'Bullet283', 'ODE' and 'Newton'.
2. Experiment number: This is an intiger between 1-6. Where 1 = 1&2, 2 = 3&4, ..., 6 = 11
3. Individual/Shared Params: 0=Shared and 1=Individual

## Results and Metrics

The results from execution main.py will be saved in the Results directory. The results are in the form of .csv files. To understand further read the code in logger.py.

**Contact**

Any questions or comments contact [Jack Collins](mailto:jack@jacktcollins.com).
