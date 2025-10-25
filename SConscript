from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add mlx90394 src files.
if GetDepend('PKG_USING_MLX90394'):
    src += Glob('src/mlx90394.c')

if GetDepend('RT_USING_SENSOR'):
    src += Glob('src/sensor_melexis_mlx90394.c')

if GetDepend('PKG_USING_MLX90394_SAMPLE'):
    src += Glob('examples/mlx90394_sample.c')

# add mlx90394 include path.
path  = [cwd + '/inc']

# add src and include to group.
group = DefineGroup('mlx90394', src, depend = ['PKG_USING_MLX90394'], CPPPATH = path)

Return('group')
