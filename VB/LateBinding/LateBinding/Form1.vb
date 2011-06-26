Public Class Form1

    Private Sub Button1_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click

        Dim mList As New List(Of Employee)
        For i = 1 To 2
            Dim m As New Manager
            m.Name = "Manager_" + i.ToString + "_Name"
            mList.Add(m)
        Next

        Dim sList As New List(Of Employee)
        For i = 1 To 2
            Dim s As New Salesman
            s.Name = "Salesman_" + i.ToString + "_Name"
            sList.Add(s)
        Next

        Dim List As New List(Of Employee)
        List.AddRange(mList)
        List.AddRange(sList)

        For Each item In List
            item.PrintMe()

            If TypeOf item Is Manager Then
                MessageBox.Show("is manager")
                DirectCast(item, Manager).ShowBoss()
            End If

            MessageBox.Show("type is " + GetType(Manager).ToString)
        Next

    End Sub

    Public Sub DoesntWork()
        Dim mList As New List(Of Manager)
        For i = 1 To 10
            Dim m As New Manager
            m.Name = "Manager_" + i.ToString + "_Name"
            mList.Add(m)
        Next

        Dim sList As New List(Of Salesman)
        For i = 1 To 10
            Dim s As New Salesman
            s.Name = "Salesman_" + i.ToString + "_Name"
            sList.Add(s)
        Next

        Dim List As New List(Of Employee)
        List.AddRange(mList)
        List.AddRange(sList)

        For Each item In List
            item.PrintMe()
        Next
    End Sub
    Public Sub Works()
        Dim m As New Manager
        m.Name = "ManagerName"

        Dim s As New Salesman
        s.Name = "SalesmanName"

        Dim Emps As New List(Of Employee)
        Emps.Add(m)
        Emps.Add(s)

        For Each item In Emps
            item.PrintMe()
        Next
    End Sub
End Class
