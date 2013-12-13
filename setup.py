from IsirPythonTools import *

package_name = 'xde_isir_controller'

setup(name='XDE-ISIRController',
		version='0.2',
		description='isir qp-based controller for xde',
		author='Soseph',
		author_email='hak@isir.upmc.fr',
		package_dir={package_name:'src'},
		packages=[package_name],
		cmdclass=cmdclass,

		script_name=script_name,
		script_args= script_args,
	)


