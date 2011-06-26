Public Class Form1

    Private Sub Button1_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click
        Dim cnt As Integer = 3
        Dim i As Integer ' loop variable
        For i = 1 To cnt
            Dim myTextBox As New TextBox
            myTextBox.Name = "createdTextBox" & i ' name the textbox 
            Me.FlowLayoutPanel1.Controls.Add(myTextBox)

        Next

        'now loop through the control collection until you find the control that you want
        'For Each control As Control In Me.FlowLayoutPanel1.Controls
        'If Control.Name = "createdTextBox2" Then
        'Control.Text = "this is a test"
        'End If
        'Next

        'or if you prefer, you can simply reference the flowlayoutpanels control collection as an array. remember, this is a 0 based array so the first control will be 0 . Consider this code:

        Me.FlowLayoutPanel1.Controls(0).Text = "this is a test"

    End Sub

    Private Sub Button2_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button2.Click


        Dim cnt As Integer = 6
        Dim i As Integer ' loop variable
        For i = 1 To cnt
            Dim myTextBox As New TextBox
            myTextBox.Name = "createdTextBox" & i ' name the textbox 
            Me.TableLayoutPanel1.Controls.Add(myTextBox)

        Next

        'now loop through the control collection until you find the control that you want
        'Dim counter As Integer = 1
        'For Each control As Control In Me.TableLayoutPanel1.Controls
        '    If control.Name = "createdTextBox" + counter.ToString Then
        '        control.Text = "test" + counter.ToString
        '    End If
        '    counter = counter + 1
        'Next

        'or if you prefer, you can simply reference the flowlayoutpanels control collection as an array. remember, this is a 0 based array so the first control will be 0 . Consider this code:
        For i = 0 To (cnt / 2) - 1
            Me.TableLayoutPanel1.Controls(i).Text = "test" + i.ToString
            Me.TableLayoutPanel1.Controls(i + 1).Text = "test" + (i + 1).ToString
        Next

        Me.TableLayoutPanel1.Controls(0).BackColor = Color.AliceBlue

    End Sub
End Class
