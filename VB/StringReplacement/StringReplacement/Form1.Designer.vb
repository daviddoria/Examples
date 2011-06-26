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
        Me.txtOriginal = New System.Windows.Forms.TextBox
        Me.txtReplacement = New System.Windows.Forms.TextBox
        Me.lblResult = New System.Windows.Forms.Label
        Me.btnReplace = New System.Windows.Forms.Button
        Me.SuspendLayout()
        '
        'txtOriginal
        '
        Me.txtOriginal.Location = New System.Drawing.Point(61, 51)
        Me.txtOriginal.Name = "txtOriginal"
        Me.txtOriginal.Size = New System.Drawing.Size(100, 20)
        Me.txtOriginal.TabIndex = 0
        Me.txtOriginal.Text = "Thing #1# to replace"
        '
        'txtReplacement
        '
        Me.txtReplacement.Location = New System.Drawing.Point(61, 105)
        Me.txtReplacement.Name = "txtReplacement"
        Me.txtReplacement.Size = New System.Drawing.Size(100, 20)
        Me.txtReplacement.TabIndex = 1
        Me.txtReplacement.Text = "Replacement"
        '
        'lblResult
        '
        Me.lblResult.AutoSize = True
        Me.lblResult.Location = New System.Drawing.Point(58, 158)
        Me.lblResult.Name = "lblResult"
        Me.lblResult.Size = New System.Drawing.Size(47, 13)
        Me.lblResult.TabIndex = 2
        Me.lblResult.Text = "lblResult"
        '
        'btnReplace
        '
        Me.btnReplace.Location = New System.Drawing.Point(86, 201)
        Me.btnReplace.Name = "btnReplace"
        Me.btnReplace.Size = New System.Drawing.Size(75, 23)
        Me.btnReplace.TabIndex = 3
        Me.btnReplace.Text = "Replace"
        Me.btnReplace.UseVisualStyleBackColor = True
        '
        'Form1
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(292, 266)
        Me.Controls.Add(Me.btnReplace)
        Me.Controls.Add(Me.lblResult)
        Me.Controls.Add(Me.txtReplacement)
        Me.Controls.Add(Me.txtOriginal)
        Me.Name = "Form1"
        Me.Text = "Form1"
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
    Friend WithEvents txtOriginal As System.Windows.Forms.TextBox
    Friend WithEvents txtReplacement As System.Windows.Forms.TextBox
    Friend WithEvents lblResult As System.Windows.Forms.Label
    Friend WithEvents btnReplace As System.Windows.Forms.Button

End Class
