from ipywidgets import Layout, Button, Box, HBox, VBox, Text, Textarea, Dropdown, Output, Accordion


def on_value_change(change):
    out.clear_output()

def on_ubuntu_change(change):
    out.clear_output()
    if change['new']=='16.04':
        ros.options=['none', 'kinetic-ros-core', 'kinetic-ros-base', 'kinetic-robot', 
                     'kinetic-perception', 'kinetic-desktop', 'kinetic-desktop-full']
        cuda.options=['none', '8.0-runtime', '8.0-devel', '9.0-runtime', '9.0-devel', 
                      '10.0-runtime', '10.0-devel']
        cudnn.options=['none', '5-runtime', '5-devel', '6-runtime', '6-devel', '7-runtime', '7-devel']
    elif change['new']=='18.04':
        ros.options=['none', 'melodic-ros-core', 'melodic-ros-base', 'melodic-robot', 
                 'melodic-perception', 'melodic-desktop', 'melodic-desktop-full']
        cuda.options=['none', '9.2-runtime', '9.2-devel', '10.0-runtime', '10.0-devel']
        cudnn.options=['none', '7-runtime', '7-devel']

def on_cuda_change(change):
    out.clear_output()
    if change['new']=='8.0-runtime' or change['new']=='8.0-devel':
        cudnn.options=['none', '5-runtime', '5-devel', '6-runtime', '6-devel', '7-runtime', '7-devel']
    else:
        cudnn.options=['none', '7-runtime', '7-devel']

name = Text(
    value='',
    placeholder='name of the Docker image',
    description='Name:',
    disabled=False
)
name.observe(on_value_change, names='value')

ubuntu = Dropdown(
    options=['16.04', '18.04'],
    value='16.04',
    description='Ubuntu:',
    disabled=False
)
ubuntu.observe(on_ubuntu_change, names='value')

x11 = Dropdown(
    options=['no', 'yes'],
    value='no',
    description='X11:',
    disabled=False
)
x11.observe(on_value_change, names='value')

opengl = Dropdown(
    options=['none', 'runtime', 'devel'],
    value='none',
    description='OpenGL:',
    disabled=False
)
opengl.observe(on_value_change, names='value')

cuda = Dropdown(
    options=['none', '8.0-runtime', '8.0-devel', '9.0-runtime', '9.0-devel', '10.0-runtime', '10.0-devel'],
    value='none',
    description='CUDA:',
    disabled=False
)
cuda.observe(on_cuda_change, names='value')

cudnn = Dropdown(
    options=['none', '5-runtime', '5-devel', '6-runtime', '6-devel', 
             '7-runtime', '7-devel'],
    value='none',
    description='cuDNN:',
    disabled=False
)
cudnn.observe(on_value_change, names='value')

ros = Dropdown(
    options=['none', 'kinetic-ros-core', 'kinetic-ros-base', 'kinetic-robot', 
             'kinetic-perception', 'kinetic-desktop', 'kinetic-desktop-full'],
    value='none',
    description='ROS:',
    disabled=False
)
ros.observe(on_value_change, names='value')

build = Dropdown(
    options=['none', 'cmake', 'catkin_make', 'catkin_build'],
    value='none',
    description='Build:',
    disabled=False
)
build.observe(on_value_change, names='value')

apt = Textarea(
    value='',
    placeholder='List of APT packages\n(one package per line)',
    description='APT:',
    disabled=False
)
apt.observe(on_value_change, names='value')

pip2 = Textarea(
    value='',
    placeholder='List of Python 2 packages\n(one package per line)',
    description='pip2:',
    disabled=False
)
pip2.observe(on_value_change, names='value')

pip3 = Textarea(
    value='',
    placeholder='List of Python 3 packages\n(one package per line)',
    description='pip3:',
    disabled=False
)
pip3.observe(on_value_change, names='value')

custom = Textarea(
    value='',
    placeholder='List of custom commands\n(one command per line)',
    description='custom:',
    disabled=False
)
custom.observe(on_value_change, names='value')

matlab = Text(
    value='',
    placeholder='path in the local host',
    description='MATLAB:',
    disabled=False
)
matlab.observe(on_value_change, names='value')

mac = Text(
    value='',
    placeholder='MAC address of local host',
    description=' ',
    disabled=False
)
mac.observe(on_value_change, names='value')

import os, yaml
from roslab_create import write_docker_file
from IPython.core.display import display, HTML
from IPython.display import clear_output

def generate_dockerfile(b):
    out.clear_output()
    with out:
        if name.value is '':
            print('Error: name is empty!')
        else:
            print("Generating Dockerfile...")
            base = {'ubuntu': ubuntu.value}
            if not opengl.value is 'none':
                base['opengl'] = opengl.value
            if not cuda.value is 'none':
                base['cuda'] = cuda.value
            if not cudnn.value is 'none':
                base['cudnn'] = cudnn.value
            if not ros.value is 'none':
                base['ros'] = ros.value
            data = {'name': name.value, 'base': base}
            if not build.value is 'none':
                data['build'] = build.value
            if not apt.value is '':
                data['apt'] = apt.value.splitlines()
            if not pip2.value is '':
                data['pip'] = pip2.value.splitlines()
            if not pip3.value is '':
                data['pip3'] = pip3.value.splitlines()
            if not custom.value is '':
                data['custom'] = custom.value.splitlines()
            if not matlab.value is '':
                data['matlab'] = {}
                data['matlab']['host_path'] = matlab.value
                data['matlab']['mac_address'] = mac.value
            if len(source_list.children) > 0:
                data['source'] = []
                for sp in source_list.children:
                    source_name = sp.children[0].value
                    repo = sp.children[1].value
                    if source_name != '' and repo != '':
                        depends = sp.children[2].value.splitlines()
                        source_build = sp.children[3].value
                        data['source'].append({'name':source_name, 'repo':repo, 'depends':depends, 'build':source_build})
                    else:
                        print('Ignoring source package with empty name and/or repo')
            with open('roslab.yaml', 'w') as outfile:
                yaml.dump(data, outfile, default_flow_style=False)
            write_docker_file(data)
            print('Done!')
            os.rename('roslab.dockerfile', 'Dockerfile')
            display(HTML('<p>It can be downloaded <a href="Dockerfile" target="_blank">from here</a>.</p>'))
            if not ros.value is 'none':
                    display(HTML('<p>Since you are using ROS, please also download <a href="ros_entrypoint.sh" target="_blank">ros_entrypoint.sh</a>.</p>'))
                    #display(HTML('<p>WARNING: some browsers deactivate the execution permissions of this file. In that case, you shoudl run the following command in a terminal:</p>'))
                    #display(HTML('''<p style="font-family:'Lucida Console', monospace">chmod a+x ros_entrypoint.sh</p>'''))
            display(HTML('<p>1. Put the downloaded file(s) in your source repository, and build your Docker image with:<br><code>docker build -t %s .</code></p>' % data['name']))
            if opengl.value is 'none' and cuda.value is 'none':
                docker_command = 'docker'
            else:
                docker_command = 'nvidia-docker'
            if x11.value is 'no':
                x11_string = ''
            else:
                x11_string = '-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix '
            if matlab.value is '':
                matlab_string = ''
            else:
                matlab_string = '-v %s:%s -v /usr/local/lib/python3.5/dist-packages/matlab:/usr/local/lib/python3.5/dist-packages/matlab --mac-address=%s ' % (matlab.value, matlab.value, mac.value)
            run_string = docker_command + ' run --rm -p 8888:8888 ' + matlab_string + x11_string + data['name']
            display(HTML('<p>2. Run your Docker image with:<br><code>%s</code></p>' % run_string))
            display(HTML('<p>3. Open this link in your browser: <a href="http://localhost:8888">http://localhost:8888</a></p>'))

generate = Button(
    description='Proceed',
    disabled=False,
    button_style='', # 'success', 'info', 'warning', 'danger' or ''
    tooltip='Generate Dockerfile'
)
generate.on_click(generate_dockerfile)

def add_source_item(b):
    name = Text(
        value='',
        placeholder='name of source package',
        description='Name:',
        disabled=False
    )
    name.observe(on_value_change, names='value')
    repo = Text(
        value='',
        placeholder='URL of git repository',
        description='repo:',
        disabled=False
    )
    repo.observe(on_value_change, names='value')
    depends = Textarea(
        value='',
        placeholder='List of APT packages\n(one package per line)',
        description='depends:',
        disabled=False
    )
    depends.observe(on_value_change, names='value')
    build = Dropdown(
        options=['cmake', 'catkin_make', 'catkin_build'],
        value='cmake',
        description='build:',
        disabled=False
    )
    build.observe(on_value_change, names='value')
    source_items = [name, repo, depends, build]
    source_form = Box(source_items, layout=Layout(
        display='flex',
        flex_flow='column',
        #border='solid 1px',
        align_items='stretch'
    ))
    source_list.children += (source_form,)
    n = len(source_list.children)
    source_list.set_title(n-1, 'Source package %d' % n)

add_source = Button(
    description='Add source package',
    disabled=False,
    button_style='', # 'success', 'info', 'warning', 'danger' or ''
    tooltip='Add source package'
)
add_source.on_click(add_source_item)

source_list = Accordion(children=[])

out = Output()

form_item_layout = Layout(
    display='flex',
    flex_flow='row',
    justify_content='space-between'
)

left_items = [name, ubuntu, x11, opengl, cuda, cudnn, ros, build, generate]
mid_items = [apt, pip2, pip3, custom, matlab, mac]
right_items = [add_source, source_list]

left_form = Box(left_items, layout=Layout(
    display='flex',
    flex_flow='column',
    #border='solid 1px',
    align_items='stretch',
    width='32%'
))
mid_form = Box(mid_items, layout=Layout(
    display='flex',
    flex_flow='column',
    #border='solid 1px',
    align_items='stretch',
    width='32%'
))
right_form = Box(right_items, layout=Layout(
    display='flex',
    flex_flow='column',
    #border='solid 1px',
    align_items='stretch',
    width='36%'
))
out.clear_output()
ui = VBox([HBox([left_form, mid_form, right_form]), out])
