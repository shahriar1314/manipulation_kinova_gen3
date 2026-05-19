# body_controller
Intro to component here

## Dependencies 

git clone https://github.com/jcalderon12/robotics-toolbox-python.git
pip install .

wget https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.6.0/kortex_api-2.6.0.post3-py3-none-any.whl

pip install kortex_api-2.6.0.post3-py3-none-any.whl

pip install protobuf==3.20.3 swift-sim numpy==1.26.0 statsmodels qpsolvers[open_source_solvers] websockets==10.4



## Configuration parameters
As any other component, *body_controller* needs a configuration file to start. In
```
etc/config
```
you can find an example of a configuration file. We can find there the following lines:
```
EXAMPLE HERE
```

## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

```
cd <body_controller's path> 
```
```
cp etc/config config
```

After editing the new config file we can run the component:

```
bin/body_controller config
```
