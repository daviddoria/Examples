Public Class Form1

    Private Sub Button1_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click
        'Dim emp As New Employee
        'emp.Name = "emp"


        Dim a As New Manager
        a.Name = "mang"


        Dim EmpList As New List(Of Employee)
        'EmpList.Add(emp)
        EmpList.Add(a)

        For Each guy In EmpList
            guy.PrintMe()
        Next


    End Sub
End Class
