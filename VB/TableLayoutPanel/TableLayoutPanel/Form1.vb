Public Class Form1

    Private Sub Button1_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click

        For i As Integer = 1 To 10 Step 2
            'setup first column

            'create a new label control, name it, and populate it with the test name
            Dim myLabelName As New Label
            myLabelName.Name = "lblTest_" + i.ToString
            myLabelName.Text = "Test " + i.ToString
            myLabelName.AutoSize = True
            Me.TableLayoutPanel1.Controls.Add(myLabelName)

            'setup second column

            'create a new label control, name it, and populate it with the test name
            'this will automatically be in the second column because the tableLayoutPanel has two columns
            'and items are added like this by default:
            '1 2
            '3 4
            '5 6
            Dim myLabelResult As New Label
            myLabelResult.Name = "lblResult_" + (i + 1).ToString
            myLabelResult.Text = "Result " + (i + 1).ToString
            myLabelResult.AutoSize = True
            Me.TableLayoutPanel1.Controls.Add(myLabelResult)
        Next

    End Sub

End Class
