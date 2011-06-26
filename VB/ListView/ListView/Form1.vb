Public Class Form1

    Private Sub Button1_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click
        'works()
        AlsoWorks()

    End Sub

    Public Sub AlsoWorks()
        ListView1.Columns.Add("Test")
        ListView1.Columns.Add("Result")

        Dim str(2) As String
        str(0) = "Test1"
        str(1) = "Result1"

        Dim itm As New ListViewItem(str)
        itm.UseItemStyleForSubItems = False
        itm.SubItems(1).ForeColor = Color.Red

        ListView1.Items.Add(itm)

    End Sub

    Public Sub works()
        'must set .View poperty to "details"

        ListView1.Columns.Add("Test")
        ListView1.Columns.Add("Result")

        Dim itm As New ListViewItem("test1")
        itm.UseItemStyleForSubItems = False
        itm.SubItems.Add("result1", Color.Red, itm.BackColor, itm.Font)

        ListView1.Items.Add(itm)
    End Sub

    Public Sub AddItems()
        'ListView1.Clear() ' delete everything including the headers

        ListView1.Items.Clear() 'only delete the grid contents

        'must set .View poperty to "details"
        ListView1.Columns.Add("Test")
        ListView1.Columns.Add("Result")

        For i As Integer = 1 To 6
            Dim str(2) As String
            Dim itm As ListViewItem
            str(0) = "Test1"
            str(1) = "Result1"

            itm = New ListViewItem(str)
            If i Mod 2 = 0 Then
                itm.ForeColor = Color.Green
            Else
                itm.ForeColor = Color.Red
            End If
            ListView1.Items.Add(itm)
        Next
    End Sub

    Private Sub btnClear_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles btnClear.Click
        ListView1.Clear()
    End Sub
End Class
