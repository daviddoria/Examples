<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()> _
Partial Class Form1
    Inherits System.Windows.Forms.Form

    'Form overrides dispose to clean up the component list.
    <System.Diagnostics.DebuggerNonUserCode()> _
    Protected Overrides Sub Dispose(ByVal disposing As Boolean)
        Try
            If disposing AndAlso components IsNot Nothing Then
                components.Dispose()
            End If
        Finally
            MyBase.Dispose(disposing)
        End Try
    End Sub

    'Required by the Windows Form Designer
    Private components As System.ComponentModel.IContainer

    'NOTE: The following procedure is required by the Windows Form Designer
    'It can be modified using the Windows Form Designer.  
    'Do not modify it using the code editor.
    <System.Diagnostics.DebuggerStepThrough()> _
    Private Sub InitializeComponent()
        Me.txtString = New System.Windows.Forms.TextBox
        Me.txtDelimiter = New System.Windows.Forms.TextBox
        Me.Label1 = New System.Windows.Forms.Label
        Me.Label2 = New System.Windows.Forms.Label
        Me.lblResult = New System.Windows.Forms.Label
        Me.btnParse = New System.Windows.Forms.Button
        Me.txtResults = New System.Windows.Forms.TextBox
        Me.SuspendLayout()
        '
        'txtString
        '
        Me.txtString.Location = New System.Drawing.Point(12, 30)
        Me.txtString.Name = "txtString"
        Me.txtString.Size = New System.Drawing.Size(259, 20)
        Me.txtString.TabIndex = 0
        Me.txtString.Text = "Enter a string."
        '
        'txtDelimiter
        '
        Me.txtDelimiter.Location = New System.Drawing.Point(12, 90)
        Me.txtDelimiter.Name = "txtDelimiter"
        Me.txtDelimiter.Size = New System.Drawing.Size(100, 20)
        Me.txtDelimiter.TabIndex = 1
        Me.txtDelimiter.Text = " "
        '
        'Label1
        '
        Me.Label1.AutoSize = True
        Me.Label1.Location = New System.Drawing.Point(9, 14)
        Me.Label1.Name = "Label1"
        Me.Label1.Size = New System.Drawing.Size(34, 13)
        Me.Label1.TabIndex = 2
        Me.Label1.Text = "String"
        '
        'Label2
        '
        Me.Label2.AutoSize = True
        Me.Label2.Location = New System.Drawing.Point(12, 74)
        Me.Label2.Name = "Label2"
        Me.Label2.Size = New System.Drawing.Size(47, 13)
        Me.Label2.TabIndex = 3
        Me.Label2.Text = "Delimiter"
        '
        'lblResult
        '
        Me.lblResult.AutoSize = True
        Me.lblResult.Location = New System.Drawing.Point(26, 128)
        Me.lblResult.Name = "lblResult"
        Me.lblResult.Size = New System.Drawing.Size(45, 13)
        Me.lblResult.TabIndex = 4
        Me.lblResult.Text = "Results:"
        '
        'btnParse
        '
        Me.btnParse.Location = New System.Drawing.Point(144, 90)
        Me.btnParse.Name = "btnParse"
        Me.btnParse.Size = New System.Drawing.Size(75, 23)
        Me.btnParse.TabIndex = 5
        Me.btnParse.Text = "Parse"
        Me.btnParse.UseVisualStyleBackColor = True
        '
        'txtResults
        '
        Me.txtResults.Location = New System.Drawing.Point(12, 144)
        Me.txtResults.Multiline = True
        Me.txtResults.Name = "txtResults"
        Me.txtResults.Size = New System.Drawing.Size(100, 114)
        Me.txtResults.TabIndex = 6
        '
        'Form1
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(296, 292)
        Me.Controls.Add(Me.txtResults)
        Me.Controls.Add(Me.btnParse)
        Me.Controls.Add(Me.lblResult)
        Me.Controls.Add(Me.Label2)
        Me.Controls.Add(Me.Label1)
        Me.Controls.Add(Me.txtDelimiter)
        Me.Controls.Add(Me.txtString)
        Me.Name = "Form1"
        Me.Text = "String Parsing"
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
    Friend WithEvents txtString As System.Windows.Forms.TextBox
    Friend WithEvents txtDelimiter As System.Windows.Forms.TextBox
    Friend WithEvents Label1 As System.Windows.Forms.Label
    Friend WithEvents Label2 As System.Windows.Forms.Label
    Friend WithEvents lblResult As System.Windows.Forms.Label
    Friend WithEvents btnParse As System.Windows.Forms.Button
    Friend WithEvents txtResults As System.Windows.Forms.TextBox

End Class
