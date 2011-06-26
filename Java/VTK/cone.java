import java.awt.BorderLayout;
import javax.swing.JFrame;
import vtk.vtkActor;
import vtk.vtkConeSource;
import vtk.vtkPanel;
import vtk.vtkPolyDataMapper;
/**
 * @author Sebastien Jourdain / Artenum
 */
public class cone {
	public static void main(String[] args) {
		// Build the VTK 3D view
		// (Should be done before using any vtk 
		//  object or load the native
		//  library first)
		vtkPanel panel3D = new vtkPanel();
		// Build a simple VTK pipeline
		vtkConeSource coneSource = new vtkConeSource();
		coneSource.SetRadius(2);
		coneSource.SetAngle(15);
		coneSource.SetHeight(2);
		coneSource.SetResolution(10);
		vtkPolyDataMapper coneMapper = new vtkPolyDataMapper();
		coneMapper.SetInput(coneSource.GetOutput());
		vtkActor coneActor = new vtkActor();
		coneActor.SetMapper(coneMapper);
		// Add actor in 3D Panel
		panel3D.GetRenderer().AddActor(coneActor);
		// Build Java Frame and include the VTK view
		JFrame frame = new JFrame("Artenum Demo");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.getContentPane().setLayout(new BorderLayout());
		frame.getContentPane().add(panel3D, BorderLayout.CENTER);
		frame.setSize(500, 500);
		frame.setLocationRelativeTo(null); // Center on desktop
		frame.setVisible(true);
	}
}
